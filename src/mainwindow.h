#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ArcTFTree.hpp"

#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <QTreeWidget>
#include <QMainWindow>
#include <QTimer>
#include <QComboBox>

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_btn_refresh_clicked();

    void on_btn_expand_all_clicked();

    void on_btn_collapse_all_clicked();

    void on_btn_view_frames_clicked();

    void on_btn_reference_clicked();

    void on_btn_target_clicked();

private:
    Ui::MainWindow *ui;

private:
    // -------------------------------------
    // ROS Components
    // -------------------------------------
    // node
    rclcpp::Node::SharedPtr node_;

    // ROS publisher
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    // ROS subscriber
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

    // ROS tf tool
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // -------------------------------------
    // Spin Function
    // -------------------------------------
    void initSpin(void);
    QTimer spin_timer_;
    // -------------------------------------
    void updateTfTreeView(QTreeWidget *widget, ARC_TF::Tree *tf_tree);
    void updateComboBox(QComboBox *cbox, std::vector<std::string> values);
    void expandTreeNextLevel(QTreeWidgetItem *item, bool next_stop = false);
};
#endif // MAINWINDOW_H
