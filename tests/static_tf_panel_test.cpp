#include <QApplication>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>
#include <QRadioButton>
#include <QClipboard>
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
    void validateAndPublishStaticTf();
    void invalidInputsAndSavedSettings();
    void heldButtonsRepeatAndStop();
    void refreshRespectsLinkChoice();
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
    QVERIFY(preview->toPlainText().contains("xyz=\"0.25 0 0\""));
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
    yaw->setValue(1.570796326795);
    const QString before = panel.findChild<QPlainTextEdit *>("urdfPreview")->toPlainText();
    panel.findChild<QPushButton *>("xyzUnit")->click();
    QCOMPARE(x->value(), 50.0);
    panel.findChild<QPushButton *>("rpyUnit")->click();
    QVERIFY(std::abs(yaw->value() - 90.0) < 1e-8);
    QCOMPARE(panel.findChild<QPlainTextEdit *>("urdfPreview")->toPlainText(), before);
    panel.findChild<QPushButton *>("copyUrdf")->click();
    QCOMPARE(QApplication::clipboard()->text(), before);
    panel.findChild<QRadioButton *>("step_0.1")->setChecked(true);
    panel.findChild<QPushButton *>("increaseX")->click();
    QVERIFY(std::abs(x->value() - 50.1) < 1e-8);
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
    auto *enabled = panel.findChild<QCheckBox *>("enableTf");
    parent->setEditText("missing"); child->setEditText("new_frame"); enabled->setChecked(true);
    QVERIFY(!enabled->isChecked());
    parent->setEditText("base"); child->setEditText("sensor"); enabled->setChecked(true);
    QVERIFY(!enabled->isChecked());
    child->setEditText("world"); enabled->setChecked(true);
    QVERIFY(!enabled->isChecked());
    child->setEditText("other_root"); enabled->setChecked(true);
    QVERIFY(enabled->isChecked());
    QVERIFY(!parent->isEnabled() && !child->isEnabled());
    enabled->setChecked(false);
    child->setEditText("new_frame");
    geometry_msgs::msg::TransformStamped received;
    auto sub = observer->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::QoS(10).transient_local().reliable(),
        [&](const tf2_msgs::msg::TFMessage &msg) { for (auto &tf : msg.transforms) if (tf.child_frame_id == "new_frame") received = tf; });
    enabled->setChecked(true);
    QVERIFY(enabled->isChecked());
    panel.findChild<QDoubleSpinBox *>("axisX")->setValue(20.0);
    panel.findChild<QDoubleSpinBox *>("axisYaw")->setValue(1.570796326795);
    QElapsedTimer timer; timer.start();
    while ((received.child_frame_id.empty() || std::abs(received.transform.translation.x-20.0) > 1e-8 || std::abs(received.transform.rotation.z-0.7071067811865476) > 1e-9) && timer.elapsed() < 5000) {
        rclcpp::spin_some(observer); QTest::qWait(10);
    }
    QCOMPARE(QString::fromStdString(received.header.frame_id), QString("base"));
    QVERIFY(std::abs(received.transform.translation.x - 20.0) < 1e-8);
    QVERIFY(std::abs(received.transform.rotation.z - 0.7071067811865476) < 1e-9);
    QVERIFY(std::abs(received.transform.rotation.w - 0.7071067811865476) < 1e-9);
    enabled->setChecked(false);
    QVERIFY(parent->isEnabled() && child->isEnabled());
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
        auto *enabled = panel.findChild<QCheckBox *>("enableTf");
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
        QVERIFY(!restored.findChild<QCheckBox *>("enableTf")->isChecked());
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
    QString xml = "<robot name=\"test\"><link name=\"base\"/>" + panel.findChild<QPlainTextEdit *>("urdfPreview")->toPlainText() + "</robot>";
    urdf::Model model;
    QVERIFY(model.initString(xml.toStdString()));
    auto joint = model.getJoint("base_to_camera_joint");
    QVERIFY(joint != nullptr);
    geometry_msgs::msg::TransformStamped received;
    auto sub = observer->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::QoS(10).transient_local().reliable(),
        [&](const tf2_msgs::msg::TFMessage &msg) { for (const auto &tf : msg.transforms) if (tf.child_frame_id == "camera") received = tf; });
    panel.findChild<QCheckBox *>("enableTf")->setChecked(true);
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
