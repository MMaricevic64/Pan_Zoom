#include <QActionGroup>
#include <QApplication>
#include <QMainWindow>
#include <QStyle>
#include <QMenuBar>
#include <QToolButton>
#include <QAction>
#include <QLabel>
#include <QFileDialog>
#include <QImageReader>
#include <QGraphicsView>
#include <QPropertyAnimation>
#include <pal/image-viewer.h>
#include "rect-selection.h"

#include <thread>
#include <iostream>
#include <fstream>
#include <ios>


class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr)
        : QMainWindow(parent)
    {
        // viewer
        viewer = new pal::ImageViewer(this);
        viewer->setText(tr("Image viewer"));
        //viewer->setToolBarMode(pal::ImageViewer::ToolBarMode::AutoHidden);

        // selection tool
        auto selecter = new pal::SelectionItem(viewer->pixmapItem());
        selecter->setVisible(false);

        auto sel = new QToolButton(this);
        sel->setToolTip(tr("Selects a rectangle area in the image"));
        sel->setIcon(QIcon(":select"));
        sel->setCheckable(true);

        // cropping only allowed when an image is visible
        auto updater = [=] {
            const bool ok = !viewer->image().isNull();
            if (ok)
                selecter->resetSelection();
            else
                sel->setChecked(false);
            sel->setEnabled(ok);
        };

        connect(sel, &QToolButton::toggled, selecter, &pal::SelectionItem::setVisible);
        connect(viewer, &pal::ImageViewer::imageChanged, selecter, updater);
        updater();
        viewer->addTool(sel);

        // file open
        QList<QByteArray> formats = QImageReader::supportedImageFormats();
        QStringList list;
        for (auto &fmt : formats)
            list.append("*." + QString(fmt));
        auto filter = QStringLiteral("Images (%1)").arg(list.join(QStringLiteral(" ")));

        auto open_action = new QAction(tr("Open an image file..."), this);
        connect(open_action, &QAction::triggered, this, [=] {
            QString path = QFileDialog::getOpenFileName(nullptr, tr("Pick an image file"),
                                                        nullptr, filter);
            if (path.isEmpty())
                return;
            viewer->setImage(QImage(path));
        });

        /* load default image */
        //viewer->setImage(QImage("large.png"));
        //viewer->setImage(QImage("../rectangle_test.png"));

        auto file_menu = menuBar()->addMenu(tr("&File"));
        file_menu->addAction(open_action);

        auto scrollbar_actions = new QActionGroup(this);
        scrollbar_actions->setExclusive(true);

        auto scrollbar_menu = menuBar()->addMenu(tr("&Scroll bars"));
        using ScrollBarPolicyMenuItem = std::pair<QString, Qt::ScrollBarPolicy>;
        for (auto item : {
                 ScrollBarPolicyMenuItem{tr("As needed"), Qt::ScrollBarAsNeeded},
                 ScrollBarPolicyMenuItem{tr("Always off"), Qt::ScrollBarAlwaysOff},
                 ScrollBarPolicyMenuItem{tr("Always on"), Qt::ScrollBarAlwaysOn},
            }) {
            auto scrollbar_action = scrollbar_menu->addAction(item.first);
            scrollbar_action->setCheckable(true);
            scrollbar_action->setChecked(viewer->view()->horizontalScrollBarPolicy() == item.second);
            connect(scrollbar_action, &QAction::triggered, this, [=] {
                viewer->view()->setHorizontalScrollBarPolicy(item.second);
                viewer->view()->setVerticalScrollBarPolicy(item.second);
                scrollbar_action->setChecked(true);
            });
            scrollbar_actions->addAction(scrollbar_action);
        }

        // fit image in window on double click
        connect(viewer->pixmapItem(), &pal::PixmapItem::doubleClicked, viewer, &pal::ImageViewer::zoomFit);

        auto rotateAnimation = new QPropertyAnimation (viewer, "rotation", this);
        rotateAnimation->setDuration(125);

        auto rotateView = [=](qreal angle) {
            rotateAnimation->setStartValue(viewer->rotation());
            rotateAnimation->setEndValue(viewer->rotation() + angle);
            rotateAnimation->start();
        };

        auto rotate_left_action = menuBar()->addAction(tr("Rotate &left"));
        connect(rotate_left_action, &QAction::triggered, [=] { rotateView(-90); });

        auto rotate_right_action = menuBar()->addAction(tr("Rotate &right"));
        connect(rotate_right_action, &QAction::triggered, [=] { rotateView(90); });

        auto reset_rotation_action = menuBar()->addAction(tr("Reset rotation"));
        connect(reset_rotation_action, &QAction::triggered, [=] { viewer->setRotation(0.); });

        auto aspect_ratio_actions = new QActionGroup(this);
        aspect_ratio_actions->setExclusive(true);

        auto aspect_ratio_menu = menuBar()->addMenu(tr("&Aspect ratio"));
        using AspectRatioMenuItem = std::pair<QString, Qt::AspectRatioMode>;
        for (auto item : {
                 AspectRatioMenuItem{tr("Keep"), Qt::KeepAspectRatio},
                 AspectRatioMenuItem{tr("Keep by expanding"), Qt::KeepAspectRatioByExpanding},
            }) {
            auto action = aspect_ratio_menu->addAction(item.first);
            action->setCheckable(true);
            action->setChecked(viewer->aspectRatioMode() == item.second);
            connect(action, &QAction::triggered, this, [=] {
                viewer->setAspectRatioMode(item.second);
                action->setChecked(true);
            });
            aspect_ratio_actions->addAction(action);
        }

        setCentralWidget(viewer);
        resize(800, 600);
    }
    pal::ImageViewer *viewer;
    ~MainWindow() override = default;

    void setImage(std::string& image_path) {
        viewer->setImage(QImage(QString::fromStdString(image_path)));
    }

    void load_test_data(std::string& filepath)
    {
        std::ifstream in(filepath, std::ios_base::in);
        if (!in) {
            std::cout << filepath << std::endl;
            std::cerr << "test data file error\n";
            std::exit(1);
        }
        viewer->test_sequence_file = filepath;

        std::string img_filepath_base, modality;
        while (in >> img_filepath_base >> modality) {
            viewer->image_paths.push_back(QString::fromStdString(img_filepath_base + ".png"));
            viewer->modalities.push_back(QString::fromStdString(modality));

            /* load test rectangle colors */
            viewer->test_box_colors.push_back({});
            std::ifstream in(img_filepath_base + ".txt", std::ios_base::in);
            if (!in) {
                std::cerr << "box_colors file error\n";
                std::exit(1);
            }
            int r = 0, g = 0, b = 0;
            while (in >> r >> g >> b) {
                viewer->test_box_colors[viewer->n_tests].push_back(QColor(r, g, b));
            }
            viewer->n_tests++;
        }
        
        viewer->setImage(QImage(viewer->image_paths[0]));
        viewer->start_button->setEnabled(true);

        QString s = QStringLiteral("0/%1 tests completed.    ")
                    .arg(viewer->n_tests);
        viewer->m_testcount_label->setText(s);
        s = QStringLiteral("%1 %2    status: 0/%3 rectangles completed")
                    .arg(viewer->image_paths[0])
                    .arg(viewer->modalities[0])
                    .arg(viewer->test_box_colors[0].size());
        viewer->setText(s);
    }
};


int main(int argc, char *argv[]) 
{
    std::string image_filepath = "large.png", box_colors_filepath;
    std::string username = "default";
    std::string log_str, test_sequence_file;
    if (argc == 2) {
        image_filepath = std::string(argv[1]);
    }
    if (argc == 3) {
        test_sequence_file = std::string(argv[1]);
        username = std::string(argv[2]);
        log_str =
            username + ", " + test_sequence_file + ", ";
    }

    QApplication a(argc, argv);
    MainWindow w;
    w.viewer->log_str = log_str;

    if (test_sequence_file != "") {
        w.load_test_data(test_sequence_file);
    } else {
        w.setImage(image_filepath);
    }

    w.show();

    /* start receving gesture events in separate thread */
    std::thread server_thread(&pal::ImageViewer::receive_events, w.viewer);
    int ret = a.exec();
    server_thread.join();
    return ret;
}

#include "main.moc"
