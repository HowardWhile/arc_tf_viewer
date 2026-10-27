#include <QApplication>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>
#include <QRadioButton>
#include <QClipboard>
#include <QContextMenuEvent>
#include <QLabel>
#include <QMenu>
#include <QLineEdit>
#include <QMouseEvent>
#include <QXmlStreamReader>
#include <QTemporaryDir>
#include <QDir>
#include <QStandardPaths>
#include <tf2_msgs/msg/tf_message.hpp>
#include <urdf/model.h>
#include <QDoubleSpinBox>
#include <QPlainTextEdit>
#include <QTest>
#include "static_tf_panel.h"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

class StaticTfPanelTest : public QObject
{
    Q_OBJECT
private slots:
    void init() { QDir(QStandardPaths::writableLocation(QStandardPaths::GenericConfigLocation) + "/arc_tf_viewer").removeRecursively(); }
    void urdfPreviewUsesMetersAndRadians();
    void unitsAndCopyPreservePose();
    void fourDecimalDisplayPreservesPose();
    void validateAndPublishStaticTf();
    void panelTextCanBeCopied();
    void newParentCanPublish();
    void clearResetsAndPublishesPose();
    void invalidInputsAndSavedSettings();
    void heldButtonsRepeatAndStop();
    void refreshRespectsLinkChoice();
    void linkDeclarationToggles();
    void mixedRpyMatchesUrdf();
    void previewRespondsToCameraControls();
};

void StaticTfPanelTest::urdfPreviewUsesMetersAndRadians()
{
    auto node = std::make_shared<rclcpp::Node>("static_tf_panel_test");
    tf2_ros::Buffer buffer(node->get_clock());
    StaticTfPanel panel(node, buffer);
    auto parent = panel.findChild<QComboBox *>("parentFrame");
    auto child = panel.findChild<QComboBox *>("childFrame");
    auto x = panel.findChild<QDoubleSpinBox *>("axisX");
    auto preview = panel.findChild<QPlainTextEdit *>("urdfPreview");
    QVERIFY(parent && child && x && preview);
    parent->setEditText("base_link");
    child->setEditText("camera_link");
    x->setValue(0.25);
    QVERIFY(preview->toPlainText().contains("xyz=\"0.2500\n               0.0000\n               0.0000\""));
    QVERIFY(preview->toPlainText().contains("parent link=\"base_link\""));
    QVERIFY(preview->toPlainText().contains("child link=\"camera_link\""));
}

void StaticTfPanelTest::unitsAndCopyPreservePose()
{
    auto node = std::make_shared<rclcpp::Node>("units_test");
    tf2_ros::Buffer buffer(node->get_clock());
    StaticTfPanel panel(node, buffer);
    panel.findChild<QComboBox *>("parentFrame")->setEditText("base");
    panel.findChild<QComboBox *>("childFrame")->setEditText("sensor");
    auto x = panel.findChild<QDoubleSpinBox *>("axisX");
    auto yaw = panel.findChild<QDoubleSpinBox *>("axisYaw");
    x->setValue(0.5);
    yaw->setValue(1.5);
    QVERIFY(panel.findChild<QPlainTextEdit *>("urdfPreview")->toPlainText().contains("rpy=\"0.0000\n               0.0000\n               1.5000\""));
    const QString before = panel.findChild<QPlainTextEdit *>("urdfPreview")->toPlainText();
    panel.findChild<QPushButton *>("xyzUnit")->click();
    QCOMPARE(x->value(), 50.0);
    panel.findChild<QPushButton *>("rpyUnit")->click();
    QVERIFY(std::abs(yaw->value() - 85.9437) < 1e-8);
    QCOMPARE(panel.findChild<QPlainTextEdit *>("urdfPreview")->toPlainText(), before);
    yaw->setValue(1.0);
    QVERIFY(panel.findChild<QPlainTextEdit *>("urdfPreview")->toPlainText().contains("rpy=\"0.0000\n               0.0000\n               0.0175\""));
    yaw->setValue(85.9437);
    panel.findChild<QPushButton *>("copyUrdf")->click();
    const QString copied = QApplication::clipboard()->text();
    QVERIFY(copied.contains("<origin xyz=\"0.5000 0.0000 0.0000\" rpy=\"0.0000 0.0000 1.5000\"/>"));
    auto *preview = panel.findChild<QPlainTextEdit *>("urdfPreview");
    preview->selectAll(); preview->copy();
    QCOMPARE(QApplication::clipboard()->text(), copied);
    panel.findChild<QRadioButton *>("step_0.1")->setChecked(true);
    panel.findChild<QPushButton *>("increaseX")->click();
    QVERIFY(std::abs(x->value() - 50.1) < 1e-8);
}

void StaticTfPanelTest::fourDecimalDisplayPreservesPose()
{
    auto node = std::make_shared<rclcpp::Node>("precision_test");
    tf2_ros::Buffer buffer(node->get_clock());
    StaticTfPanel panel(node, buffer);
    auto *x = panel.findChild<QDoubleSpinBox *>("axisX");
    auto *unit = panel.findChild<QPushButton *>("xyzUnit");
    QCOMPARE(x->decimals(), 4);
    unit->click(); unit->click();
    x->setValue(0.0001);
    const QString before = panel.findChild<QPlainTextEdit *>("urdfPreview")->toPlainText();
    unit->click();
    QCOMPARE(x->text(), QString("0.0000"));
    QCOMPARE(panel.findChild<QPlainTextEdit *>("urdfPreview")->toPlainText(), before);
    unit->click(); unit->click();
    QCOMPARE(x->value(), 0.0001);
    QCOMPARE(panel.findChild<QPlainTextEdit *>("urdfPreview")->toPlainText(), before);
}

void StaticTfPanelTest::validateAndPublishStaticTf()
{
    auto node = std::make_shared<rclcpp::Node>("publish_test");
    auto observer = std::make_shared<rclcpp::Node>("static_tf_observer");
    tf2_ros::Buffer buffer(node->get_clock());
    geometry_msgs::msg::TransformStamped fixture;
    fixture.header.stamp = node->now();
    fixture.header.frame_id = "world"; fixture.child_frame_id = "base";
    fixture.transform.rotation.w = 1.0;
    QVERIFY(buffer.setTransform(fixture, "fixture", true));
    fixture.header.frame_id = "other_root"; fixture.child_frame_id = "sensor";
    QVERIFY(buffer.setTransform(fixture, "fixture", true));
    StaticTfPanel panel(node, buffer);
    auto *parent = panel.findChild<QComboBox *>("parentFrame");
    auto *child = panel.findChild<QComboBox *>("childFrame");
    auto *enabled = panel.findChild<QPushButton *>("enableTf");
    parent->setEditText(""); child->setEditText("new_frame"); enabled->setChecked(true);
    QVERIFY(!enabled->isChecked());
    parent->setEditText("base"); child->setEditText("sensor"); enabled->setChecked(true);
    QVERIFY(!enabled->isChecked());
    child->setEditText("world"); enabled->setChecked(true);
    QVERIFY(!enabled->isChecked());
    child->setEditText("other_root"); enabled->setChecked(true);
    QVERIFY(enabled->isChecked());
    QVERIFY(parent->lineEdit()->isReadOnly() && child->lineEdit()->isReadOnly());
    parent->lineEdit()->selectAll(); parent->lineEdit()->copy();
    QCOMPARE(QApplication::clipboard()->text(), QString("base"));
    QTest::keyClick(parent, Qt::Key_Down);
    QCOMPARE(parent->currentText(), QString("base"));
    enabled->setChecked(false);
    child->setEditText("new_frame");
    geometry_msgs::msg::TransformStamped received;
    auto sub = observer->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::QoS(10).transient_local().reliable(),
        [&](const tf2_msgs::msg::TFMessage &msg) { for (auto &tf : msg.transforms) if (tf.child_frame_id == "new_frame") received = tf; });
    enabled->setChecked(true);
    QVERIFY(enabled->isChecked());
    panel.findChild<QDoubleSpinBox *>("axisX")->setValue(20.0);
    panel.findChild<QDoubleSpinBox *>("axisYaw")->setValue(1.5708);
    QElapsedTimer timer; timer.start();
    while ((received.child_frame_id.empty() || std::abs(received.transform.translation.x-20.0) > 1e-8 || std::abs(received.transform.rotation.z-0.7071080798594735) > 1e-9) && timer.elapsed() < 5000) {
        rclcpp::spin_some(observer); QTest::qWait(10);
    }
    QCOMPARE(QString::fromStdString(received.header.frame_id), QString("base"));
    QVERIFY(std::abs(received.transform.translation.x - 20.0) < 1e-8);
    QVERIFY(std::abs(received.transform.rotation.z - 0.7071080798594735) < 1e-9);
    QVERIFY(std::abs(received.transform.rotation.w - 0.7071054825112363) < 1e-9);
    enabled->setChecked(false);
    QVERIFY(!parent->lineEdit()->isReadOnly() && !child->lineEdit()->isReadOnly());
    panel.findChild<QDoubleSpinBox *>("axisX")->setValue(42.0);
    QElapsedTimer quiet; quiet.start();
    while (quiet.elapsed() < 300) { rclcpp::spin_some(observer); QTest::qWait(10); }
    QVERIFY(std::abs(received.transform.translation.x - 20.0) < 1e-8);
    enabled->setChecked(true);
    QVERIFY(enabled->isChecked());
    timer.restart();
    while (std::abs(received.transform.translation.x - 42.0) > 1e-8 && timer.elapsed() < 5000) {
        rclcpp::spin_some(observer); QTest::qWait(10);
    }
    QVERIFY(std::abs(received.transform.translation.x - 42.0) < 1e-8);
    (void)sub;
}

void StaticTfPanelTest::panelTextCanBeCopied()
{
    auto node = std::make_shared<rclcpp::Node>("copy_text_test");
    tf2_ros::Buffer buffer(node->get_clock());
    StaticTfPanel panel(node, buffer);
    panel.show();
    auto *status = panel.findChild<QLabel *>("status");
    QVERIFY(status->textInteractionFlags() & Qt::TextSelectableByMouse);
    status->setSelection(0, 8);
    status->setFocus();
    QTest::keyClick(status, Qt::Key_C, Qt::ControlModifier);
    QCOMPARE(QApplication::clipboard()->text(), QString("Disabled"));
    auto *button = panel.findChild<QPushButton *>("clearPose");
    QVERIFY(button);
    QContextMenuEvent context(QContextMenuEvent::Mouse, button->rect().center(), button->mapToGlobal(button->rect().center()));
    QApplication::sendEvent(button, &context);
    auto *menu = qobject_cast<QMenu *>(QApplication::activePopupWidget());
    QVERIFY(menu);
    QVERIFY(!menu->actions().isEmpty());
    menu->actions().first()->trigger();
    QCOMPARE(QApplication::clipboard()->text(), QString("Clear pose"));
    menu->close();
    auto *input = panel.findChild<QComboBox *>("parentFrame")->lineEdit();
    input->setText("base_link"); input->selectAll(); input->copy();
    QCOMPARE(QApplication::clipboard()->text(), QString("base_link"));
    panel.findChild<QComboBox *>("childFrame")->setEditText("camera_link");
    auto *preview = panel.findChild<QWidget *>("tfPreview");
    QContextMenuEvent previewContext(QContextMenuEvent::Mouse, preview->rect().center(), preview->mapToGlobal(preview->rect().center()));
    QApplication::sendEvent(preview, &previewContext);
    menu = qobject_cast<QMenu *>(QApplication::activePopupWidget());
    QVERIFY(menu);
    QCOMPARE(menu->actions().size(), 2);
    menu->actions().last()->trigger();
    QCOMPARE(QApplication::clipboard()->text(), QString("camera_link"));
    menu->close();
}

void StaticTfPanelTest::newParentCanPublish()
{
    auto node = std::make_shared<rclcpp::Node>("new_parent_test");
    auto observer = std::make_shared<rclcpp::Node>("new_parent_observer");
    tf2_ros::Buffer buffer(node->get_clock());
    StaticTfPanel panel(node, buffer);
    auto *parent = panel.findChild<QComboBox *>("parentFrame");
    auto *child = panel.findChild<QComboBox *>("childFrame");
    auto *enabled = panel.findChild<QPushButton *>("enableTf");
    geometry_msgs::msg::TransformStamped received;
    auto sub = observer->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::QoS(10).transient_local().reliable(),
        [&](const tf2_msgs::msg::TFMessage &msg) { for (const auto &tf : msg.transforms) received = tf; });
    QVERIFY(enabled->isCheckable());
    QVERIFY(enabled->text().isEmpty());
    QCOMPARE(enabled->width(), enabled->height());
    QVERIFY(!enabled->icon().isNull());
    QCOMPARE(enabled->accessibleName(), QString("Enable static TF"));
    const auto playIcon = enabled->icon().pixmap(32, 32).toImage();
    parent->setEditText("new_root"); child->setEditText("new_child");
    enabled->setChecked(true);
    QCOMPARE(enabled->accessibleName(), QString("Disable static TF"));
    QVERIFY(enabled->icon().pixmap(32, 32).toImage() != playIcon);
    QVERIFY(enabled->isChecked());
    QElapsedTimer timer; timer.start();
    while (received.child_frame_id != "new_child" && timer.elapsed() < 5000) { rclcpp::spin_some(observer); QTest::qWait(10); }
    QCOMPARE(QString::fromStdString(received.header.frame_id), QString("new_root"));
    QCOMPARE(QString::fromStdString(received.child_frame_id), QString("new_child"));
    enabled->setChecked(false);
    QCOMPARE(enabled->accessibleName(), QString("Enable static TF"));
    QCOMPARE(enabled->icon().pixmap(32, 32).toImage(), playIcon);
    geometry_msgs::msg::TransformStamped existing;
    existing.header.stamp = node->now(); existing.header.frame_id = "existing_root"; existing.child_frame_id = "leaf";
    existing.transform.rotation.w = 1.0;
    QVERIFY(buffer.setTransform(existing, "fixture", true));
    parent->setEditText("another_new_root"); child->setEditText("existing_root");
    enabled->setChecked(true);
    QVERIFY(enabled->isChecked());
    timer.restart();
    while (received.child_frame_id != "existing_root" && timer.elapsed() < 5000) { rclcpp::spin_some(observer); QTest::qWait(10); }
    QCOMPARE(QString::fromStdString(received.header.frame_id), QString("another_new_root"));
    QCOMPARE(QString::fromStdString(received.child_frame_id), QString("existing_root"));
    (void)sub;
}

void StaticTfPanelTest::clearResetsAndPublishesPose()
{
    auto node = std::make_shared<rclcpp::Node>("clear_pose_test");
    auto observer = std::make_shared<rclcpp::Node>("clear_pose_observer");
    tf2_ros::Buffer buffer(node->get_clock());
    StaticTfPanel panel(node, buffer);
    auto *parent = panel.findChild<QComboBox *>("parentFrame");
    auto *child = panel.findChild<QComboBox *>("childFrame");
    auto *enabled = panel.findChild<QPushButton *>("enableTf");
    parent->setEditText("new_root"); child->setEditText("new_child");
    const char *axes[] = {"axisX", "axisY", "axisZ", "axisRoll", "axisPitch", "axisYaw"};
    for (const auto *name : axes) panel.findChild<QDoubleSpinBox *>(name)->setValue(1.0);
    panel.findChild<QPushButton *>("xyzUnit")->click();
    panel.findChild<QRadioButton *>("step_0.1")->setChecked(true);
    geometry_msgs::msg::TransformStamped received;
    auto sub = observer->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::QoS(10).transient_local().reliable(),
        [&](const tf2_msgs::msg::TFMessage &msg) { for (const auto &tf : msg.transforms) received = tf; });
    enabled->setChecked(true);
    QVERIFY(enabled->isChecked());
    auto *clear = panel.findChild<QPushButton *>("clearPose");
    QVERIFY(clear != nullptr);
    panel.findChild<QDoubleSpinBox *>("axisX")->findChild<QLineEdit *>()->setText("-");
    clear->click();
    QVERIFY(panel.findChild<QDoubleSpinBox *>("axisX")->findChild<QLineEdit *>()->hasAcceptableInput());
    for (const auto *name : axes) QCOMPARE(panel.findChild<QDoubleSpinBox *>(name)->value(), 0.0);
    QCOMPARE(parent->currentText(), QString("new_root"));
    QCOMPARE(child->currentText(), QString("new_child"));
    QCOMPARE(panel.findChild<QPushButton *>("xyzUnit")->text(), QString("cm"));
    QVERIFY(panel.findChild<QRadioButton *>("step_0.1")->isChecked());
    QElapsedTimer timer; timer.start();
    while ((received.child_frame_id.empty() || std::abs(received.transform.rotation.w - 1.0) > 1e-10) && timer.elapsed() < 5000) {
        rclcpp::spin_some(observer); QTest::qWait(10);
    }
    QCOMPARE(QString::fromStdString(received.child_frame_id), QString("new_child"));
    QCOMPARE(received.transform.translation.x, 0.0);
    QCOMPARE(received.transform.translation.y, 0.0);
    QCOMPARE(received.transform.translation.z, 0.0);
    QCOMPARE(received.transform.rotation.w, 1.0);
    (void)sub;
}

void StaticTfPanelTest::invalidInputsAndSavedSettings()
{
    auto node = std::make_shared<rclcpp::Node>("settings_test");
    tf2_ros::Buffer buffer(node->get_clock());
    geometry_msgs::msg::TransformStamped fixture;
    fixture.header.stamp = node->now(); fixture.header.frame_id = "world"; fixture.child_frame_id = "base"; fixture.transform.rotation.w = 1;
    QVERIFY(buffer.setTransform(fixture, "fixture", true));
    fixture.header.frame_id = "other"; fixture.child_frame_id = "leaf";
    QVERIFY(buffer.setTransform(fixture, "fixture", true));
    {
        StaticTfPanel panel(node, buffer);
        panel.show();
        auto *parent = panel.findChild<QComboBox *>("parentFrame");
        auto *child = panel.findChild<QComboBox *>("childFrame");
        parent->setEditText("base"); child->setEditText("other");
        QVERIFY(!panel.findChild<QCheckBox *>("includeChildLink")->isChecked());
        auto *x = panel.findChild<QDoubleSpinBox *>("axisX");
        x->setValue(2000000000.0);
        panel.findChild<QPushButton *>("xyzUnit")->click();
        panel.findChild<QPushButton *>("xyzUnit")->click();
        QCOMPARE(x->value(), 2000000000000.0);
        auto *input = x->findChild<QLineEdit *>();
        input->setText("-");
        auto *enabled = panel.findChild<QPushButton *>("enableTf");
        enabled->setChecked(true);
        QVERIFY(!enabled->isChecked());
        input->setText("2000000000000");
        enabled->setChecked(true);
        QVERIFY(enabled->isChecked());
        panel.close();
        QVERIFY(enabled->isChecked());
        panel.show();
        QVERIFY(enabled->isChecked());
        auto text = panel.findChild<QPlainTextEdit *>("urdfPreview")->toPlainText();
        QXmlStreamReader xml("<robot>" + text + "</robot>");
        while (!xml.atEnd()) xml.readNext();
        QVERIFY(!xml.hasError());
    }
    {
        StaticTfPanel restored(node, buffer);
        QVERIFY(!restored.findChild<QPushButton *>("enableTf")->isChecked());
        QVERIFY(!restored.findChild<QCheckBox *>("includeChildLink")->isChecked());
        QCOMPARE(restored.findChild<QComboBox *>("parentFrame")->currentText(), QString("base"));
        QCOMPARE(restored.findChild<QComboBox *>("childFrame")->currentText(), QString("other"));
        QCOMPARE(restored.findChild<QDoubleSpinBox *>("axisX")->value(), 2000000000000.0);
        QCOMPARE(restored.findChild<QPushButton *>("xyzUnit")->text(), QString("mm"));
    }
}

void StaticTfPanelTest::heldButtonsRepeatAndStop()
{
    auto node = std::make_shared<rclcpp::Node>("repeat_test");
    tf2_ros::Buffer buffer(node->get_clock());
    StaticTfPanel panel(node, buffer);
    panel.show();
    auto *plus = panel.findChild<QPushButton *>("increaseX");
    auto *x = panel.findChild<QDoubleSpinBox *>("axisX");
    QTest::mousePress(plus, Qt::LeftButton);
    QTest::qWait(700);
    QTest::mouseRelease(plus, Qt::LeftButton);
    QVERIFY(x->value() > 0.01);
    const double released = x->value();
    QTest::qWait(200);
    QCOMPARE(x->value(), released);
}

void StaticTfPanelTest::refreshRespectsLinkChoice()
{
    auto node = std::make_shared<rclcpp::Node>("refresh_test");
    tf2_ros::Buffer buffer(node->get_clock());
    StaticTfPanel panel(node, buffer);
    panel.findChild<QComboBox *>("childFrame")->setEditText("late_root");
    auto *refresh = panel.findChild<QPushButton *>("refreshFrames");
    QVERIFY(refresh->text().isEmpty());
    QVERIFY(!refresh->icon().isNull());
    QCOMPARE(refresh->size().width(), refresh->size().height());
    QCOMPARE(refresh->accessibleName(), QString("Refresh frames"));
    auto *include = panel.findChild<QCheckBox *>("includeChildLink");
    QVERIFY(include->isChecked());
    geometry_msgs::msg::TransformStamped fixture;
    fixture.header.stamp = node->now(); fixture.header.frame_id = "late_root"; fixture.child_frame_id = "leaf"; fixture.transform.rotation.w = 1;
    QVERIFY(buffer.setTransform(fixture, "fixture", true));
    panel.findChild<QPushButton *>("refreshFrames")->click();
    QVERIFY(!include->isChecked());
    include->click();
    QVERIFY(include->isChecked());
    panel.findChild<QPushButton *>("refreshFrames")->click();
    QVERIFY(include->isChecked());
}

void StaticTfPanelTest::linkDeclarationToggles()
{
    auto node = std::make_shared<rclcpp::Node>("link_toggles_test");
    tf2_ros::Buffer buffer(node->get_clock());
    geometry_msgs::msg::TransformStamped fixture;
    fixture.header.stamp = node->now(); fixture.header.frame_id = "world"; fixture.child_frame_id = "base"; fixture.transform.rotation.w = 1;
    QVERIFY(buffer.setTransform(fixture, "fixture", true));
    StaticTfPanel panel(node, buffer);
    auto *parent = panel.findChild<QComboBox *>("parentFrame");
    auto *child = panel.findChild<QComboBox *>("childFrame");
    auto *parentLink = panel.findChild<QCheckBox *>("includeParentLink");
    auto *childLink = panel.findChild<QCheckBox *>("includeChildLink");
    QVERIFY(parentLink && childLink);
    QCOMPARE(parentLink->text(), QString("Parent link"));
    QCOMPARE(childLink->text(), QString("Child link"));
    QVERIFY(!panel.findChild<QPlainTextEdit *>("urdfPreview")->toPlainText().contains("<link name=\"\"/>"));
    parent->setEditText("world"); child->setEditText("sensor");
    QVERIFY(!parentLink->isChecked());
    QVERIFY(childLink->isChecked());
    auto *preview = panel.findChild<QPlainTextEdit *>("urdfPreview");
    QVERIFY(!preview->toPlainText().contains("<link name=\"world\"/>"));
    QVERIFY(preview->toPlainText().contains("<link name=\"sensor\"/>"));
    parentLink->click(); childLink->click();
    QVERIFY(preview->toPlainText().contains("<link name=\"world\"/>"));
    QVERIFY(!preview->toPlainText().contains("<link name=\"sensor\"/>"));
    panel.findChild<QPushButton *>("copyUrdf")->click();
    QVERIFY(QApplication::clipboard()->text().contains("<link name=\"world\"/>"));
    QVERIFY(!QApplication::clipboard()->text().contains("<link name=\"sensor\"/>"));
    panel.findChild<QPushButton *>("refreshFrames")->click();
    QVERIFY(parentLink->isChecked());
    QVERIFY(!childLink->isChecked());
    parent->setEditText("new_root");
    QVERIFY(parentLink->isChecked());
    child->setEditText("base");
    QVERIFY(!childLink->isChecked());
}

void StaticTfPanelTest::mixedRpyMatchesUrdf()
{
    auto node = std::make_shared<rclcpp::Node>("mixed_pose_test");
    auto observer = std::make_shared<rclcpp::Node>("mixed_pose_observer");
    tf2_ros::Buffer buffer(node->get_clock());
    geometry_msgs::msg::TransformStamped fixture;
    fixture.header.stamp = node->now(); fixture.header.frame_id = "world"; fixture.child_frame_id = "base"; fixture.transform.rotation.w = 1;
    QVERIFY(buffer.setTransform(fixture, "fixture", true));
    StaticTfPanel panel(node, buffer);
    panel.findChild<QComboBox *>("parentFrame")->setEditText("base");
    panel.findChild<QComboBox *>("childFrame")->setEditText("camera");
    panel.findChild<QDoubleSpinBox *>("axisX")->setValue(0.2);
    panel.findChild<QDoubleSpinBox *>("axisY")->setValue(-0.3);
    panel.findChild<QDoubleSpinBox *>("axisZ")->setValue(1.1);
    panel.findChild<QDoubleSpinBox *>("axisRoll")->setValue(0.5);
    panel.findChild<QDoubleSpinBox *>("axisPitch")->setValue(-0.25);
    panel.findChild<QDoubleSpinBox *>("axisYaw")->setValue(1.2);
    panel.findChild<QPushButton *>("copyUrdf")->click();
    QString xml = "<robot name=\"test\"><link name=\"base\"/>" + QApplication::clipboard()->text() + "</robot>";
    urdf::Model model;
    QVERIFY(model.initString(xml.toStdString()));
    auto joint = model.getJoint("base_to_camera_joint");
    QVERIFY(joint != nullptr);
    geometry_msgs::msg::TransformStamped received;
    auto sub = observer->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::QoS(10).transient_local().reliable(),
        [&](const tf2_msgs::msg::TFMessage &msg) { for (const auto &tf : msg.transforms) if (tf.child_frame_id == "camera") received = tf; });
    panel.findChild<QPushButton *>("enableTf")->setChecked(true);
    QElapsedTimer timer; timer.start();
    while (received.child_frame_id.empty() && timer.elapsed() < 5000) { rclcpp::spin_some(observer); QTest::qWait(10); }
    QVERIFY(!received.child_frame_id.empty());
    const auto &p = joint->parent_to_joint_origin_transform.position;
    const auto &q = joint->parent_to_joint_origin_transform.rotation;
    QVERIFY(std::abs(received.transform.translation.x - p.x) < 1e-10);
    QVERIFY(std::abs(received.transform.translation.y - p.y) < 1e-10);
    QVERIFY(std::abs(received.transform.translation.z - p.z) < 1e-10);
    const auto &r = received.transform.rotation;
    const double dot = r.x*q.x + r.y*q.y + r.z*q.z + r.w*q.w;
    QVERIFY(std::abs(std::abs(dot) - 1.0) < 1e-10);
    (void)sub;
}

void StaticTfPanelTest::previewRespondsToCameraControls()
{
    auto node = std::make_shared<rclcpp::Node>("camera_test");
    tf2_ros::Buffer buffer(node->get_clock());
    StaticTfPanel panel(node, buffer);
    panel.findChild<QComboBox *>("parentFrame")->setEditText("base");
    panel.findChild<QComboBox *>("childFrame")->setEditText("camera");
    panel.findChild<QDoubleSpinBox *>("axisX")->setValue(1.0);
    panel.show();
    auto *view = panel.findChild<QWidget *>("tfPreview");
    QVERIFY(view != nullptr);
    QTest::qWait(300);
    const QImage first = view->grab().toImage();
    QTest::qWait(500);
    QVERIFY(first != view->grab().toImage());
    QTest::mousePress(view, Qt::LeftButton, Qt::NoModifier, QPoint(100,100));
    QMouseEvent move(QEvent::MouseMove, QPointF(150,120), Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(view, &move);
    QTest::mouseRelease(view, Qt::LeftButton, Qt::NoModifier, QPoint(150,120));
    QTest::qWait(1100);
    const QImage settled = view->grab().toImage();
    panel.findChild<QPushButton *>("autoOrbit")->click();
    QTest::qWait(600);
    QVERIFY(settled != view->grab().toImage());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    QTemporaryDir config;
    qputenv("XDG_CONFIG_HOME", config.path().toUtf8());
    QApplication app(argc, argv);
    StaticTfPanelTest test;
    const int result = QTest::qExec(&test, argc, argv);
    rclcpp::shutdown();
    return result;
}
#include "static_tf_panel_test.moc"
