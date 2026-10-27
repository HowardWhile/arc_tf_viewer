#ifndef STATIC_TF_PANEL_H
#define STATIC_TF_PANEL_H

#include <QDialog>
#include <array>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>

class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QCheckBox;
class QLabel;
class TfPreview;

class StaticTfPanel : public QDialog
{
    Q_OBJECT
public:
    StaticTfPanel(rclcpp::Node::SharedPtr node, tf2_ros::Buffer &buffer, QWidget *parent = nullptr);
    ~StaticTfPanel() override;
    void refreshFrames();
private:
    void updateOutput();
    void updateUnits();
    void updateStep(double step);
    void saveSettings();
    void loadSettings();
    void setEnabledPublishing(bool enabled);
    QString validateFrames() const;
    bool inputsValid() const;
    void publishTransform();
    std::map<QString, QString> frameParents() const;
    double displayFactor(int index) const;
    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer &buffer_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
    QComboBox *parentFrame_, *childFrame_;
    QLineEdit *jointName_;
    QPlainTextEdit *urdfPreview_;
    QCheckBox *enable_, *includeLink_;
    QLabel *status_;
    QPushButton *xyzUnit_, *rpyUnit_;
    TfPreview *preview_;
    std::array<QDoubleSpinBox *, 6> axes_{};
    std::array<double, 6> values_{};
    int xyzUnitIndex_ = 0, rpyUnitIndex_ = 0;
    double step_ = 0.01;
    bool updating_ = false, loading_ = false, jointEdited_ = false, includeLinkEdited_ = false;
    std::map<QString, QString> published_;
};
#endif
