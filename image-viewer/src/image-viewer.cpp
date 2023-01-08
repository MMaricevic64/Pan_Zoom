#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <mutex>
#include <QApplication>
#include <QEnterEvent>
#include <QGraphicsScene>
#include <QGraphicsSceneHoverEvent>
#include <QGraphicsView>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QScrollBar>
#include <QToolButton>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QPushButton>
#include "pal/image-viewer.h"

#include <string>

#ifdef __linux__
  /* socket related - Linux*/
  #include <sys/socket.h>
  #include <unistd.h>
  #include <sys/un.h>

#elif _WIN32
  /* socket related - Windows*/
  #include <winsock2.h>
  #include <windows.h>
  #include <ws2tcpip.h>
  #include "afunix.h"
  #include <filesystem>
#endif

//MSVC compiler
#ifdef _MSC_VER
  #pragma comment(lib, "Ws2_32.lib")
#endif

#include <string>

#include <thread>
#include <chrono>

static void init_image_viewer_resource() {
    // This must be done outside of any namespace
    Q_INIT_RESOURCE(image_viewer);
}

namespace pal {

enum msg_type {
  MSG_MOVE = 1,
  MSG_ZOOM,
};

enum pan_state {
  PAN_START = 1,
  PAN_ACTIVE,
  PAN_STOP
};

/* message struct for sending to the image-viewer over socket */
struct gesture_msg {
  unsigned type;
  float dx;
  float dy;
  bool zoom_in;
  unsigned pan_state;
};

//Global variable for socket path and socket
#ifdef __linux__
  std::string server_filename = "/tmp/image-viewer-server.sock";
  int sockfd;

#elif _WIN32
  std::string server_socket;
  SOCKET Socket = INVALID_SOCKET;

#endif

// Graphics View with better mouse events handling
class GraphicsView : public QGraphicsView {
    Q_OBJECT
public:
    explicit GraphicsView(ImageViewer *viewer)
        : QGraphicsView()
        , m_viewer(viewer)
    {
        static std::once_flag inititialized;
        std::call_once(inititialized, init_image_viewer_resource);

        // no antialiasing or filtering, we want to see the exact image content
        setRenderHint(QPainter::Antialiasing, false);
        setDragMode(QGraphicsView::ScrollHandDrag);
        setOptimizationFlags(QGraphicsView::DontSavePainterState);
        setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
        setTransformationAnchor(QGraphicsView::AnchorUnderMouse); // zoom at cursor position
        setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        setInteractive(true);
        setMouseTracking(true);

    }
    float scene_x_range;
    float scene_y_range;
    bool pan_active;

public:
    void wheelEvent(QWheelEvent *event) override {
        const auto d = event->angleDelta();

        if (event->modifiers() == Qt::NoModifier) {
            if (d.x() > 0 || d.y() > 0)
                m_viewer->zoomIn(3);
            else if (d.x() < 0 || d.y() < 0)
                m_viewer->zoomOut(3);
            event->accept();
        }
        else
            QGraphicsView::wheelEvent(event);
    }

    void enterEvent(EnterEvent *event) override {
        QGraphicsView::enterEvent(event);
        viewport()->setCursor(Qt::CrossCursor);
    }

    void mousePressEvent(QMouseEvent *event) override {
        auto scene_top_left = mapToScene(QPoint(0, 0));
        auto scene_bottom_right = mapToScene(QPoint(viewport()->width(), viewport()->height()));

        /* range of pixels shown on the screen at once
           (using the current zoom level) */
        scene_y_range = scene_bottom_right.y() - scene_top_left.y();
        scene_x_range = scene_bottom_right.x() - scene_top_left.x();

        QGraphicsView::mousePressEvent(event);
    }

    void mouseMoveEvent(QMouseEvent *event) override {
        if (pan_active) {
            QGraphicsView::mouseMoveEvent(event);
        }
    }

    void mouseReleaseEvent(QMouseEvent *event) override {
        QGraphicsView::mouseReleaseEvent(event);
        viewport()->setCursor(Qt::CrossCursor);
    }

private:
	#ifdef __linux__
		int sockfd;
	#elif _WIN32
		SOCKET Socket = INVALID_SOCKET;
	#endif
    ImageViewer *m_viewer;

private slots:
};

void ImageViewer::receive_events() {

	#ifdef __linux__
	    /* setup socket for receiving hand gesture events. */
	    struct sockaddr_un server_addr;
	    memset(&server_addr, 0, sizeof(server_addr));
	    server_addr.sun_family = AF_UNIX;
	    strcpy(server_addr.sun_path, server_filename.c_str());

	    sockfd = socket(AF_UNIX, SOCK_DGRAM, 0);

	    int bind_result = bind(sockfd, (struct sockaddr *) &server_addr, sizeof(server_addr));
      	if (bind_result < 0 || sockfd < 0) goto Exit; //Socket or Bind failed

	#elif _WIN32
    	int Result = 0;

    	SOCKADDR_UN ServerAddr = { 0 };
  		WSADATA WsaData = { 0 };

  		// Get Temp path on Windows
  		std::filesystem::path server_socket_path = std::filesystem::temp_directory_path() / "image-viewer-server.sock";
  		server_socket = server_socket_path.string();

  		// Replace all occurrences of character '\' with '/'
  		std::replace(server_socket.begin(), server_socket.end(), '\\', '/');

  		// Initialize Winsock
  		Result = WSAStartup(MAKEWORD(2,2), &WsaData);
 		  if (Result != 0) {
      		std::cerr<<"WSAStartup failed with error: " << Result << "\n";
          goto Exit;
  		}

  		// Create a AF_UNIX stream server socket.
		Socket = socket(AF_UNIX, SOCK_STREAM, 0);
		if (Socket == INVALID_SOCKET) {
		    std::cerr<<"Socket failed with error: " << WSAGetLastError() << "\n";
        goto Exit;
		}

		// Set server address
		ServerAddr.sun_family = AF_UNIX;
		strcpy(ServerAddr.sun_path, server_socket.c_str());

		// Bind the socket to the path.
	    Result = bind(Socket, (struct sockaddr *)&ServerAddr, sizeof(ServerAddr));
	    if (Result == SOCKET_ERROR) {
	        std::cerr<<"Bind failed with error: " << WSAGetLastError() << "\n";
	        goto Exit;
	    }

	    // Listen to start accepting connections.
	    Result = listen(Socket, SOMAXCONN);
	    if (Result == SOCKET_ERROR) {
	        std::cerr<<"Listen failed with error: " << WSAGetLastError() << "\n";
	        goto Exit;
	    }

	    std::cerr<<"Accepting connections on: " << server_socket << "\n";
    	// Accept a connection from mediapipe (client).
	    Socket = accept(Socket, NULL, NULL);
	    if (Socket == INVALID_SOCKET) {
	        std::cerr<<"Accept failed with error: " << WSAGetLastError() << "\n";
      		goto Exit;
	    }
      else{
          std::cerr<< "Accepted a connection to mediapipe (client)." << "\n";
      }

	#endif

    while (true) {
        /* receive gesture events and emit signals to GUI */
        struct gesture_msg event;
        #ifdef __linux__
        	recv(sockfd, &event, sizeof(event), 0);
        #elif _WIN32
        	recv(Socket, (char *) &event, sizeof(event), 0);
        #endif
        if (event.type == MSG_ZOOM) {
            if (event.zoom_in) {
                zoomIn(3);
            } else {
                zoomOut(3);
            }
        } else if (event.type == MSG_MOVE) {
            switch (event.pan_state) {
                case PAN_START: {
                    m_view->pan_active = true;
                    /* emit mouse click event */

                    /* find center of the screen (in image coordinates) */
                    auto center = m_view->viewport()->rect().center();
                    /* Simulate mouse press (emit event) */ // possible mem leak, use unique pointer?
                    QMouseEvent *press_event = new QMouseEvent(
                        QEvent::MouseButtonPress, center,
                        Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
                    m_view->mousePressEvent(press_event); //
                    //QCoreApplication::postEvent(m_view, press_event);
                    this->prev_position = center;
                    break;
                }
                case PAN_ACTIVE: {
                    const float PAN_SCALING_FACTOR = 1.8f;
                    QPointF offset = {event.dx * m_view->viewport()->width() * PAN_SCALING_FACTOR,
                                      event.dy * m_view->viewport()->height() * PAN_SCALING_FACTOR};
                    QPointF new_position = this->prev_position + offset;


                    /* emit mouse move event */
                    QMouseEvent *move_event = new QMouseEvent(
                        QEvent::MouseMove, new_position,
                        Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
                    m_view->mouseMoveEvent(move_event);

                    this->prev_position = new_position;
                    break;
                }
                case PAN_STOP: {
                    /* emit mouse release event */
                    QMouseEvent *release_event = new QMouseEvent(
                        QEvent::MouseButtonRelease, {500, 500}, // change value to actual center?
                        Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
                    m_view->mouseReleaseEvent(release_event); //
                    m_view->pan_active = false;

                    break;
                }
                default:
                    std::cout << "this should never happen (pan event with unspecified state)\n";
                    break;
            }
        }
    }

    Exit:
        #ifdef __linux__
          ::close(sockfd);
          /* release unix socket if it exists */
          unlink(server_filename.c_str());
        #elif _WIN32
        // Socket cleanup
          if (Socket != INVALID_SOCKET) {
            closesocket(Socket);
          }
          DeleteFileA(server_socket.c_str());
          WSACleanup();
    	#endif
}

ImageViewer::ImageViewer(QWidget *parent)
    : QFrame(parent)
    , m_zoom_level(0)
    , m_fit(true)
    , m_bar_mode(ToolBarMode::Visible)
    , m_aspect_ratio_mode(Qt::KeepAspectRatio)
{
    auto scene = new QGraphicsScene(this);
    m_view = new GraphicsView(this);
    m_view->setScene(scene);

    // graphic object holding the image buffer
    m_pixmap = new PixmapItem;
    scene->addItem(m_pixmap);
    connect(m_pixmap, &PixmapItem::mouseMoved, this, &ImageViewer::mouseAt);
    connect(m_pixmap, &PixmapItem::sizeChanged, this, &ImageViewer::updateSceneRect);

    makeToolbar();
    start_button->setEnabled(false);

    auto box = new QVBoxLayout;
    box->setContentsMargins(5,0,5,0);
    box->addWidget(m_toolbar);
    box->addWidget(m_view, 1);
    setLayout(box);

    /* periodically check colors in corners to track experiment progress */
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(check_corner_colors()));
    QObject::connect(exit_button, SIGNAL(clicked()), this, SLOT(exit_viewer()));
    QObject::connect(start_button, SIGNAL(clicked()), this, SLOT(start_test_timer()));
    QObject::connect(next_button, SIGNAL(clicked()), this, SLOT(next_test()));
    timer->start(10);
}

void ImageViewer::check_corner_colors()
{
    if (!this->experiment_active) {
        return;
    }
    if (this->completed_tests == this->test_box_colors.size()) {
        return; /* test is completed */
    }

    std::array<QPointF, 4> viewport_corners = {
        m_view->mapToScene(QPoint(0, 0)),
        m_view->mapToScene(QPoint(m_view->viewport()->width(), 0)),
        m_view->mapToScene(QPoint(0, m_view->viewport()->height())),
        m_view->mapToScene(QPoint(m_view->viewport()->width(), m_view->viewport()->height()))
    };

    bool box_covers_view = true; /* does the current goal rectangle cover the whole screen? */

    /* check color in each screen corner */
    QRgb current_box_color =
        this->test_box_colors[this->completed_tests][this->completed_boxes].rgb();
    for (QPointF& corner : viewport_corners) {
        if (!m_pixmap->image().valid(corner.x(), corner.y())) {
            box_covers_view = false;
            break;
        }
        QRgb corner_color = m_pixmap->image().pixel(corner.x(), corner.y());
        if (corner_color != current_box_color) {
            box_covers_view = false;
            break;
        }
    }

    if (box_covers_view) {
        this->completed_boxes++;
        QString s = QStringLiteral("%1, %2    status: %3/%4 rectangles completed")
                        .arg(this->image_paths[this->completed_tests])
                        .arg(this->modalities[this->completed_tests])
                        .arg(this->completed_boxes)
                        .arg(this->test_box_colors[this->completed_tests].size());
        m_text_label->setText(s);
        //Sound feedback when rectangle is completed
        #ifdef __linux__
        	std::cout << "\a" << "\n";
		#elif _WIN32
			Beep(523, 300); // 523 hertz (C5) for 300 milliseconds, this function is part of Windows.h   
		#endif
        if (this->completed_boxes == this->test_box_colors[this->completed_tests].size()) {
            /* test is completed */
            auto end_time = std::chrono::high_resolution_clock::now();
            auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - this->start_time).count();

            /* output test results in csv format (username, modality, filename, time) */
            std::cout
              << this->log_str
              << this->image_paths[this->completed_tests].toStdString() + ", "
              << this->modalities[this->completed_tests].toStdString() + ", "
              << std::to_string(milliseconds) << std::endl;

            this->completed_tests++;
            this->completed_boxes = 0; /* reset for next test image */
            QString s = QStringLiteral("%1/%2 tests completed    ")
                    .arg(this->completed_tests)
                    .arg(this->n_tests);
            this->m_testcount_label->setText(s);
            this->start_button->setEnabled(false);
            if (this->completed_tests == this->n_tests) {
                s = QStringLiteral("Test finished, time: %1 milliseconds. Test sequence completed.")
                        .arg(milliseconds);
                this->next_button->setEnabled(false);
            } else {
                s = QStringLiteral("Test finished, time: %1 milliseconds")
                        .arg(milliseconds);
                this->next_button->setEnabled(true);
            }
            this->experiment_active = false;
            m_teststatus_label->setText(s);

        }
    }
    //std::cout << qRed(rgb) << " " << qGreen(rgb) << " " << qBlue(rgb) << "\n";
}

void ImageViewer::start_test_timer()
{
    start_time = std::chrono::high_resolution_clock::now();
    m_teststatus_label->setText("Test started.");
    start_button->setEnabled(false);
    experiment_active = true;
    QString s = QStringLiteral("%1, %2    status: %3/%4 rectangles completed")
                        .arg(this->image_paths[this->completed_tests])
                        .arg(this->modalities[this->completed_tests])
                        .arg(this->completed_boxes)
                        .arg(this->test_box_colors[this->completed_tests].size());
    m_text_label->setText(s);
}

void ImageViewer::exit_viewer()
{
    #ifdef __linux__
      ::close(sockfd);
      /* release unix socket if it exists */
      unlink(server_filename.c_str());

    #elif _WIN32
      // Socket cleanup
      if (Socket != INVALID_SOCKET) {
        closesocket(Socket);
      }
      DeleteFileA(server_socket.c_str());
      WSACleanup();

    #endif
    std::exit(0);
}

void ImageViewer::next_test()
{
    this->setImage(QImage(this->image_paths[this->completed_tests]));
    this->zoomFit();
    next_button->setEnabled(false);
    start_button->setEnabled(true);
    QString s = QStringLiteral("%1, %2    status: %3/%4 rectangles completed")
                        .arg(this->image_paths[this->completed_tests])
                        .arg(this->modalities[this->completed_tests])
                        .arg(this->completed_boxes)
                        .arg(this->test_box_colors[this->completed_tests].size());
    m_text_label->setText(s);
}

// toolbar with a few quick actions and display information
void ImageViewer::makeToolbar() {
    // text and value at pixel
    m_text_label = new QLabel(this);
    m_text_label->setStyleSheet(QStringLiteral("QLabel { font-weight: bold; }"));
    m_testcount_label = new QLabel(this);
    m_testcount_label->setStyleSheet(QStringLiteral("QLabel { font-weight: bold; }"));
    m_teststatus_label = new QLabel(this);
    m_teststatus_label->setStyleSheet(QStringLiteral("QLabel { color: blue; }"));
    m_pixel_value = new QLabel(this);

    auto fit = new QToolButton(this);
    fit->setToolTip(tr("Fit image to window"));
    fit->setIcon(QIcon(":zoom-fit"));
    connect(fit, &QToolButton::clicked, this, &ImageViewer::zoomFit);

    auto orig = new QToolButton(this);
    orig->setToolTip(tr("Resize image to its original size"));
    orig->setIcon(QIcon(":zoom-1"));
    connect(orig, &QToolButton::clicked, this, &ImageViewer::zoomOriginal);

    start_button = new QPushButton("Start", this);
    exit_button = new QPushButton("Exit", this);
    next_button = new QPushButton("Next test", this);
    next_button->setEnabled(false);

    m_toolbar = new QWidget;
    auto box = new QHBoxLayout(m_toolbar);
    m_toolbar->setContentsMargins(0,0,0,0);
    box->setContentsMargins(0,0,0,0);
    box->addWidget(m_testcount_label);
    box->addWidget(m_text_label);
    box->addWidget(start_button);
    box->addWidget(next_button);
    box->addWidget(m_teststatus_label);
    box->addStretch(1);
    box->addWidget(exit_button);
    box->addWidget(m_pixel_value);
    box->addWidget(fit);
    box->addWidget(orig);
}

QString ImageViewer::text() const {
    return m_text_label->text();
}

void ImageViewer::setText(const QString &txt) {
    m_text_label->setText(txt);
}

const QImage &ImageViewer::image() const {
    return m_pixmap->image();
}

void ImageViewer::setImage(const QImage &im) {
    m_pixmap->setImage(im);

    if (m_fit)
        zoomFit();

    emit imageChanged();
}

void ImageViewer::setAspectRatioMode(Qt::AspectRatioMode aspect_ratio_mode) {
    m_aspect_ratio_mode = aspect_ratio_mode;
    if (m_fit)
        zoomFit();
}

const PixmapItem *ImageViewer::pixmapItem() const {
    return m_pixmap;
}

PixmapItem *ImageViewer::pixmapItem() {
    return m_pixmap;
}

ImageViewer::ToolBarMode ImageViewer::toolBarMode() const {
    return m_bar_mode;
}

void ImageViewer::setToolBarMode(ToolBarMode mode) {
    m_bar_mode = mode;
    if (mode == ToolBarMode::Hidden)
        m_toolbar->hide();
    else if (mode == ToolBarMode::Visible)
        m_toolbar->show();
    else
        m_toolbar->setVisible(underMouse());
}

bool ImageViewer::isAntialiasingEnabled() const {
    return m_view->renderHints() & QPainter::Antialiasing;
}

void ImageViewer::enableAntialiasing(bool on) {
    m_view->setRenderHint(QPainter::Antialiasing, on);
}

QGraphicsView *ImageViewer::view() const {
    return m_view;
}

void ImageViewer::addTool(QWidget *tool) {
    m_toolbar->layout()->addWidget(tool);
}

Qt::AspectRatioMode ImageViewer::aspectRatioMode() const {
    return m_aspect_ratio_mode;
}

qreal ImageViewer::rotation() const {
    return 180. * rotationRadians() / M_PI;
}

qreal ImageViewer::rotationRadians() const {
    auto p10 = m_view->transform().map(QPointF(1., 0.));
    return std::atan2(p10.y(), p10.x());
}

void ImageViewer::setRotation(qreal angle) {
    m_view->rotate(angle - rotation());
    if (m_fit)
        zoomFit();
}

qreal ImageViewer::scale() const {
    auto square = [](qreal value) { return value * value; };
    return std::sqrt(square(m_view->transform().m11()) + square(m_view->transform().m12()));
}

void ImageViewer::setMatrix() {
    qreal newScale = std::pow(2.0, m_zoom_level / 10.0);

    QTransform mat;
    mat.scale(newScale, newScale);
    mat.rotateRadians(rotationRadians());

    m_view->setTransform(mat);
    emit zoomChanged(scale());
}

void ImageViewer::zoomFit() {
    /* Fit in view by KeepAspectRatioByExpanding does not keep the position
     * find out the current viewport center move back to that position after
     * fitting. It is done here instead of inside the resize event handler
     * because fitInView may be triggered from a number of events, not just
     * the resize event.
     */
    auto cr = QRect(m_view->viewport()->rect().center(), QSize(2, 2));
    auto cen = m_view->mapToScene(cr).boundingRect().center();

    m_view->fitInView(m_pixmap, m_aspect_ratio_mode);
    m_zoom_level = int(10.0 * std::log2(scale()));
    m_fit = true;

    if (m_aspect_ratio_mode == Qt::KeepAspectRatioByExpanding)
        m_view->centerOn(cen);

    emit zoomChanged(scale());
}

void ImageViewer::zoomOriginal() {
    m_zoom_level = 0;
    m_fit = false;
    setMatrix();
}

void ImageViewer::zoomIn(int level) {
    m_zoom_level += level;
    m_fit = false;
    setMatrix();
}

void ImageViewer::zoomOut(int level) {
    m_zoom_level -= level;
    m_fit = false;
    setMatrix();
}

void ImageViewer::mouseAt(int x, int y) {
    if (m_pixmap->image().valid(x,y)) {
        QRgb rgb = m_pixmap->image().pixel(x, y);
        auto s = QStringLiteral("[%1, %2] (%3, %4, %5)")
                    .arg(x)
                    .arg(y)
                    .arg(qRed(rgb))
                    .arg(qGreen(rgb))
                    .arg(qBlue(rgb));
        m_pixel_value->setText(s);
    }
    else
        m_pixel_value->setText(QString());
}

void ImageViewer::updateSceneRect(int w, int h) {
    Q_UNUSED(w)
    Q_UNUSED(h)
    m_view->scene()->setSceneRect(m_pixmap->boundingRect());
}

void ImageViewer::enterEvent(EnterEvent *event) {
    QFrame::enterEvent(event);
    if (m_bar_mode == ToolBarMode::AutoHidden) {
        m_toolbar->show();
        if (m_fit)
            zoomFit();
    }
}

void ImageViewer::leaveEvent(QEvent *event) {
    QFrame::leaveEvent(event);
    if (m_bar_mode == ToolBarMode::AutoHidden) {
        m_toolbar->hide();
        if (m_fit)
            zoomFit();
    }
}

void ImageViewer::resizeEvent(QResizeEvent *event) {
    QFrame::resizeEvent(event);
    if (m_fit)
        zoomFit();
}

void ImageViewer::showEvent(QShowEvent *event) {
    QFrame::showEvent(event);
    if (m_fit)
        zoomFit();
}


PixmapItem::PixmapItem(QGraphicsItem *parent) :
    QObject(), QGraphicsPixmapItem(parent)
{
    setAcceptHoverEvents(true);
}

void PixmapItem::setImage(QImage im) {
    if (im.isNull()) {
        m_image.fill(Qt::white);
        im = m_image.copy();
    }
    std::swap(m_image, im);

    setPixmap(QPixmap::fromImage(m_image));

    if (m_image.size() != im.size())
        emit sizeChanged(m_image.width(), m_image.height());

    emit imageChanged(m_image);
}

void PixmapItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event) {
    auto pos = event->pos();
    emit doubleClicked(int(pos.x()), int(pos.y()));
    QGraphicsItem::mouseDoubleClickEvent(event);
}

void PixmapItem::mousePressEvent(QGraphicsSceneMouseEvent *event) {
    QGraphicsItem::mousePressEvent(event);
}

void PixmapItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
    QGraphicsItem::mouseReleaseEvent(event);
}

void PixmapItem::hoverMoveEvent(QGraphicsSceneHoverEvent *event) {
    auto pos = event->pos();
    emit mouseMoved(int(pos.x()), int(pos.y()));
    QGraphicsItem::hoverMoveEvent(event);
}

} // namespace pal

#include "image-viewer.moc"
