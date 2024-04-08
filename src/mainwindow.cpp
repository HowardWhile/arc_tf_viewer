#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <yaml-cpp/yaml.h>
#include <QDebug>

#define ROS_PRINT(...) RCLCPP_INFO(this->node_->get_logger(), __VA_ARGS__)

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    // -------------------------------------
    // QT Initialization
    // -------------------------------------
    ui->setupUi(this);

    // -------------------------------------
    // ROS Initialization
    // -------------------------------------
    rclcpp::init(0, nullptr);
    this->node_ = rclcpp::Node::make_shared("ui");

    // -------------------------------------
    // ROS publisher
    // -------------------------------------
    this->publisher_ = this->node_->create_publisher<std_msgs::msg::String>("ros2qt_pub", 10);

    // -------------------------------------
    // ROS subscriber
    // -------------------------------------
    this->subscriber_ = node_->create_subscription<std_msgs::msg::String>(
        "ros2qt_sub", 10,
        [&](const std_msgs::msg::String::SharedPtr msg)
        {
            // Handle received message
            QString receivedMsg = QString::fromStdString(msg->data);
            RCLCPP_INFO(this->node_->get_logger(), "Subscribe message: %s", msg->data.c_str());
        });

    // -------------------------------------
    // ROS tf tool
    // -------------------------------------
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->node_->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // -------------------------------------
    this->initSpin();
}

MainWindow::~MainWindow()
{
    // Stop the spin timer
    this->spin_timer_.stop();

    // Shutdown ROS
    rclcpp::shutdown();

    delete ui;
}

void MainWindow::initSpin(void)
{
    // Initialize spinning with a timer interval of 1 ms
    this->spin_timer_.setInterval(1);
    QObject::connect(&this->spin_timer_,
                     &QTimer::timeout,
                     [&]()
                     {
                         // Spin ROS for handling callbacks
                         rclcpp::spin_some(node_);
                     });
    this->spin_timer_.start();
}

void MainWindow::updateTfTreeView(QTreeWidget *widget, ARC_TF::Tree *tf_tree)
{
    // 清除原有的項目
    widget->clear();

    // 獲取所有根節點
    auto roots = tf_tree->getRoots();

    // 使用迭代方式處理每個節點及其子節點
    for (const auto root : roots)
    {
        QTreeWidgetItem *rootItem = new QTreeWidgetItem(widget);
        rootItem->setText(0, QString::fromStdString(root->value));

        std::vector<std::pair<QTreeWidgetItem *, const ARC_TF::TreeNode *>> stack;
        stack.push_back({rootItem, root});

        while (!stack.empty())
        {
            auto [parentItem, node] = stack.back();
            stack.pop_back();

            for (const auto child : node->children)
            {
                QTreeWidgetItem *childItem = new QTreeWidgetItem(parentItem);
                childItem->setText(0, QString::fromStdString(child->value));
                parentItem->addChild(childItem);
                stack.push_back({childItem, child});
            }
        }

        widget->addTopLevelItem(rootItem);
    }
}

void MainWindow::on_btn_pub_clicked()
{
    std::string text = ui->tbox_pub_msg->text().toStdString();
    RCLCPP_INFO(this->node_->get_logger(), "Publish message: %s", text.c_str());

    std_msgs::msg::String msg;
    msg.data = text;

    // Publish the ROS message
    publisher_->publish(msg);
}

void MainWindow::on_pushButton_clicked()
{
    //    RCLCPP_INFO(this->node_->get_logger(), "CallFramesAsString: %s", tf_buffer_->allFramesAsString().c_str());
    //    RCLCPP_INFO(this->node_->get_logger(), "allFramesAsYAML: %s", tf_buffer_->allFramesAsYAML().c_str());
    //    std::vector<std::string> names = tf_buffer_->getAllFrameNames();
    //    RCLCPP_INFO(node_->get_logger(), "All TF Frames:");
    //    for (const auto& name : names) {
    //        RCLCPP_INFO(node_->get_logger(), "- %s", name.c_str());
    //    }

    YAML::Node yaml_node = YAML::Load(tf_buffer_->allFramesAsYAML().c_str());
    // 使用 YAML::Emitter 將 YAML::Node 轉換為 YAML 字串
    //    YAML::Emitter emitter;
    //    emitter << yaml_node;
    // 打印 YAML 字串
    //    std::cout << "YAML Node Content:\n" << emitter.c_str() << std::endl;

    ROS_PRINT("TF Struct---");

    ARC_TF::Tree tf_tree;
    std::map<std::string, std::string> tf_parent_map;
    for (const auto &pair : yaml_node)
    {
        std::string tf_name = pair.first.as<std::string>();
        YAML::Node value_node = pair.second;
        std::string tf_parent = value_node["parent"].as<std::string>();

        tf_tree.addChild(tf_parent, tf_name);
    }

    ROS_PRINT("\r\n%s", tf_tree.toString().c_str());
    this->updateTfTreeView(ui->treeWidget, &tf_tree);
    ui->treeWidget->expandAll();

    //    std::cout << tree.toString() << std::endl;
}
