#pragma once

#include <QMainWindow>
#include <scene_editor_window.h>

/**
 * Класс главного окна
 */
class SceneEditorMainWindow : public QMainWindow {
Q_OBJECT

public:
    /**
     * Конструктор
     */
    SceneEditorMainWindow();

private slots:

    void onNewScene();

    void onOpenScene();

    void onSaveScene();

private:
    SceneEditorWindow *_window;
};
