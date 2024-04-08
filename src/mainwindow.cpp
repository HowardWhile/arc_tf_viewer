#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <yaml-cpp/yaml.h>
#include <QProcess>
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
//    this->publisher_ = this->node_->create_publisher<std_msgs::msg::String>("ros2qt_pub", 10);

    // -------------------------------------
    // ROS subscriber
    // -------------------------------------
//    this->subscriber_ = node_->create_subscription<std_msgs::msg::String>(
//        "ros2qt_sub", 10,
//        [&](const std_msgs::msg::String::SharedPtr msg)
//        {
//            // Handle received message
//            QString receivedMsg = QString::fromStdString(msg->data);
//            RCLCPP_INFO(this->node_->get_logger(), "Subscribe message: %s", msg->data.c_str());
//        });

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

void MainWindow::on_btn_refresh_clicked()
{
     YAML::Node yaml_node = YAML::Load(tf_buffer_->allFramesAsYAML().c_str());

     ARC_TF::Tree tf_tree;
     for (const auto &pair : yaml_node)
     {
         std::string tf_name = pair.first.as<std::string>();
         YAML::Node value_node = pair.second;
         std::string tf_parent = value_node["parent"].as<std::string>();

         tf_tree.addChild(tf_parent, tf_name);
     }
     ROS_PRINT("TF Struct---");
     ROS_PRINT("\r\n%s", tf_tree.toString().c_str());

     this->updateTfTreeView(ui->treeWidget, &tf_tree);
     ui->treeWidget->expandAll();
}


void MainWindow::on_btn_expand_all_clicked()
{
    ui->treeWidget->expandAll();
}


void MainWindow::on_btn_collapse_all_clicked()
{
    ui->treeWidget->collapseAll();
}

void MainWindow::on_btn_graphic_clicked()
{
    // 创建 QProcess 对象
    QProcess* process = new QProcess(this);

    // 设置要执行的命令和参数
    QString command = "rqt_graph";
    QStringList arguments;

    // 启动进程并执行命令
    process->start(command, arguments);

    // 检查进程是否成功启动
    if (!process->waitForStarted()) {
        qDebug() << "Failed to start command.";
    }
}

