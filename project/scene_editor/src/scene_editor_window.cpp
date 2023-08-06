#include "scene_editor_window.h"
#include "scene_editor_main_window.h"

/**
 * Конструктор
 * @param mw главное окно
 * @param scenePath путь к описанию сцены
 */
SceneEditorWindow::SceneEditorWindow(SceneEditorMainWindow *mw, const std::string &scenePath)
        : mainWindow(mw) {

    glWidget = new SceneEditorWidget(mw);

    auto *mainLayout = new QVBoxLayout;
    auto *container = new QHBoxLayout;

    auto *w = new QWidget;
    w->setLayout(container);
    mainLayout->addWidget(w);

    container->addWidget(glWidget);

    auto *btnContainer = new QHBoxLayout;
    auto *btnWidget = new QWidget;
    btnWidget->setLayout(btnContainer);
    mainLayout->addWidget(btnWidget);


    dockBtn = new QPushButton(tr("Undock"), this);
    connect(dockBtn, SIGNAL(clicked()), this, SLOT(dockUndock()));
    btnContainer->addWidget(dockBtn);


    auto *editContainer = new QVBoxLayout;
    auto *editWidget = new QWidget;
    editWidget->setLayout(editContainer);

    container->addWidget(editWidget);

    auto *firstEditButtonContainer = new QHBoxLayout;
    auto *firstEditButtonWidget = new QWidget;
    firstEditButtonWidget->setLayout(firstEditButtonContainer);


    container->addWidget(editWidget);

    editContainer->addWidget(firstEditButtonWidget);

    addBtn = new QPushButton(tr("Add"), this);
    connect(addBtn, SIGNAL(clicked()), this, SLOT(doAdd()));
    firstEditButtonContainer->addWidget(addBtn);

    deleteBtn = new QPushButton(tr("Delete"), this);
    connect(deleteBtn, SIGNAL(clicked()), this, SLOT(doDelete()));
    firstEditButtonContainer->addWidget(deleteBtn);

    translateLabel = new QLabel(this);
    translateLabel->setText("Translate");
    translateLabel->setAlignment(Qt::AlignCenter);
    editContainer->addWidget(translateLabel);

    translateXSlider = createSlider(-500, 500);
    QWidget *translateWidgetX = getSliderWithCaption(translateXSlider, editContainer, "X");
    editContainer->addWidget(translateWidgetX);

    translateYSlider = createSlider(-500, 500);
    QWidget *translateWidgetY = getSliderWithCaption(translateYSlider, editContainer, "Y");
    editContainer->addWidget(translateWidgetY);

    translateZSlider = createSlider(-500, 500);
    QWidget *translateWidgetZ = getSliderWithCaption(translateZSlider, editContainer, "Z");
    editContainer->addWidget(translateWidgetZ);


    rotateLabel = new QLabel(this);
    rotateLabel->setText("Rotate");
    rotateLabel->setAlignment(Qt::AlignCenter);
    editContainer->addWidget(rotateLabel);

    rotateXSlider = createSlider(-360, 360);
    QWidget *rotateWidgetX = getSliderWithCaption(rotateXSlider, editContainer, "R");
    editContainer->addWidget(rotateWidgetX);

    rotateYSlider = createSlider(-360, 360);
    QWidget *rotateWidgetY = getSliderWithCaption(rotateYSlider, editContainer, "P");
    editContainer->addWidget(rotateWidgetY);

    rotateZSlider = createSlider(-360, 360);
    QWidget *rotateWidgetZ = getSliderWithCaption(rotateZSlider, editContainer, "Y");
    editContainer->addWidget(rotateWidgetZ);

    scaleLabel = new QLabel(this);
    scaleLabel->setText("Scale");
    scaleLabel->setAlignment(Qt::AlignCenter);
    editContainer->addWidget(scaleLabel);


    scaleXSlider = createSlider(1, 10000);
    QWidget *scaleWidgetX = getSliderWithCaption(scaleXSlider, editContainer, "X");
    editContainer->addWidget(scaleWidgetX);

    scaleYSlider = createSlider(1, 10000);
    QWidget *scaleWidgetY = getSliderWithCaption(scaleYSlider, editContainer, "Y");
    editContainer->addWidget(scaleWidgetY);

    scaleZSlider = createSlider(1, 10000);
    QWidget *scaleWidgetZ = getSliderWithCaption(scaleZSlider, editContainer, "Z");
    editContainer->addWidget(scaleWidgetZ);

    connect(translateXSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setScrollTranslationX(int)));
    connect(glWidget, SIGNAL(setScrollTranslationXChanged(int)), translateXSlider, SLOT(setValue(int)));

    connect(translateYSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setScrollTranslationY(int)));
    connect(glWidget, SIGNAL(setScrollTranslationYChanged(int)), translateYSlider, SLOT(setValue(int)));

    connect(translateZSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setScrollTranslationZ(int)));
    connect(glWidget, SIGNAL(setScrollTranslationZChanged(int)), translateZSlider, SLOT(setValue(int)));


    connect(rotateXSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setScrollRotationX(int)));
    connect(glWidget, SIGNAL(setScrollRotationXChanged(int)), rotateXSlider, SLOT(setValue(int)));

    connect(rotateYSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setScrollRotationY(int)));
    connect(glWidget, SIGNAL(setScrollRotationYChanged(int)), rotateYSlider, SLOT(setValue(int)));

    connect(rotateZSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setScrollRotationZ(int)));
    connect(glWidget, SIGNAL(setScrollRotationZChanged(int)), rotateZSlider, SLOT(setValue(int)));


    connect(scaleXSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setScrollScaleX(int)));
    connect(glWidget, SIGNAL(setScrollScaleXChanged(int)), scaleXSlider, SLOT(setValue(int)));

    connect(scaleYSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setScrollScaleY(int)));
    connect(glWidget, SIGNAL(setScrollScaleYChanged(int)), scaleYSlider, SLOT(setValue(int)));

    connect(scaleZSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setScrollScaleZ(int)));
    connect(glWidget, SIGNAL(setScrollScaleZChanged(int)), scaleZSlider, SLOT(setValue(int)));


    auto *secondEditButtonContainer = new QHBoxLayout;
    auto *secondEditButtonWidget = new QWidget;
    secondEditButtonWidget->setLayout(secondEditButtonContainer);

    editContainer->addWidget(secondEditButtonWidget);

    prevBtn = new QPushButton(tr("Prev"), this);
    connect(prevBtn, SIGNAL(clicked()), this, SLOT(doPrev()));
    secondEditButtonContainer->addWidget(prevBtn);

    nextBtn = new QPushButton(tr("Next"), this);
    connect(nextBtn, SIGNAL(clicked()), this, SLOT(doNext()));
    secondEditButtonContainer->addWidget(nextBtn);


    std::shared_ptr<bmpf::Scene> sceneWrapper = std::make_shared<bmpf::Scene>();
    if (!scenePath.empty())
        sceneWrapper->loadFromFile(scenePath);


    _pathFinder = std::make_shared<bmpf::OneDirectionPathFinder>(sceneWrapper, true, 100, 10, 4000, 1, 2);


    glWidget->setSliders(std::vector<QSlider *>{
            translateXSlider, translateYSlider, translateZSlider,
            rotateXSlider, rotateYSlider, rotateZSlider,
            scaleXSlider, scaleYSlider, scaleZSlider
    });

    glWidget->setPathFinder(_pathFinder);


    QDesktopWidget dw;

    glWidget->setFixedSize(1000, 700);

    setLayout(mainLayout);
    setWindowTitle(tr("Scene Editor"));
}

/**
 * Создать слайдер
 * @param min минимальное значение
 * @param max максимальное значение
 * @return новый слайдер
 */
QSlider *SceneEditorWindow::createSlider(int min, int max) {
    auto *slider = new QSlider(Qt::Horizontal);
    slider->setRange(min, max);
    return slider;
}

/**
 * Обработчик события клавиатуры
 * @param event событие
 */
void SceneEditorWindow::keyPressEvent(QKeyEvent *e) {
    if (e->key() == Qt::Key_Escape)
        close();
    else
        QWidget::keyPressEvent(e);
}

/**
 * Открепить окно
 */
void SceneEditorWindow::dockUndock() {
    if (parent()) {
        setParent(nullptr);
        setAttribute(Qt::WA_DeleteOnClose);
        move(QApplication::desktop()->width() / 2 - width() / 2,
             QApplication::desktop()->height() / 2 - height() / 2);
        dockBtn->setText(tr("Dock"));
        show();
    } else {
        if (!mainWindow->centralWidget()) {
            if (mainWindow->isVisible()) {
                setAttribute(Qt::WA_DeleteOnClose, false);
                dockBtn->setText(tr("Undock"));
                mainWindow->setCentralWidget(this);
            } else {
                QMessageBox::information(nullptr, tr("Cannot dock"), tr("Main window already closed"));
            }
        } else {
            QMessageBox::information(nullptr, tr("Cannot dock"), tr("Main window already occupied"));
        }
    }
}

/**
 * Добавить робота
 */
void SceneEditorWindow::doAdd() {
    QString fileName = QFileDialog::getOpenFileName(
            this, QString::fromUtf8("Открыть файл"), ""
    );

    glWidget->doAdd(fileName.toStdString());

}

/**
 * Удалить робота
 */
void SceneEditorWindow::doDelete() {
    glWidget->doDelete();
}

/**
 * Получить слайдер с заголовком
 * @param pSlider слайдер
 * @param pLayout разметка
 * @param string заголовок
 * @return слайдер с заголовком
 */
QWidget *SceneEditorWindow::getSliderWithCaption(QSlider *pSlider, QVBoxLayout *pLayout, const char *caption) {
    auto *sliderLayout = new QHBoxLayout;
    auto *sliderWidget = new QWidget;
    sliderWidget->setLayout(sliderLayout);

    sliderLayout->addWidget(pSlider);

    auto *sliderLabel = new QLabel(this);
    sliderLabel->setText(caption);
    sliderLayout->addWidget(sliderLabel);

    return sliderWidget;

}

/**
 * Следующий объект
 */
void SceneEditorWindow::doNext() {
    glWidget->doNext();
}

/**bmpf::
 * Предыдущий объект
 */
void SceneEditorWindow::doPrev() {
    glWidget->doPrev();
}

/**
 * Сохранить сцену
 * @param path путь к файлу
 */
void SceneEditorWindow::saveScene(const std::string &path) {
    _pathFinder->getScene()->saveToFile(path);
}


