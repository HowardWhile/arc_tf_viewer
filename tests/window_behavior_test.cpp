#include "mainwindow.h"
#include "static_tf_panel.h"
#include <QApplication>
#include <QCheckBox>
#include <QPushButton>
#include <QTest>
#include <QWindow>

class WindowBehaviorTest : public QObject
{
    Q_OBJECT
private slots:
    void editorIsIndependentAndReopens();
};

void WindowBehaviorTest::editorIsIndependentAndReopens()
{
    MainWindow main;
    main.show();
    auto *button = main.findChild<QPushButton *>("staticTfButton");
    QVERIFY(button);
    button->click();
    StaticTfPanel *panel = nullptr;
    for (auto *window : QApplication::topLevelWidgets()) {
        if (auto *candidate = qobject_cast<StaticTfPanel *>(window)) panel = candidate;
    }
    QVERIFY(panel);
    QVERIFY(panel->isVisible());
    QVERIFY(panel->windowHandle());
    QVERIFY(!panel->windowHandle()->transientParent());
    auto *includeLink = panel->findChild<QCheckBox *>("includeChildLink");
    if (!includeLink->isChecked()) includeLink->click();
    includeLink->click();
    panel->close();
    QVERIFY(!panel->isVisible());
    button->click();
    QVERIFY(panel->isVisible());
    QVERIFY(!includeLink->isChecked());
    main.close();
    QVERIFY(!panel->isVisible());
}

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    WindowBehaviorTest test;
    return QTest::qExec(&test, argc, argv);
}
#include "window_behavior_test.moc"
