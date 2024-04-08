#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <yaml-cpp/yaml.h>

#include <QProcess>
#include <QDebug>
#include <QCoreApplication>
#include <QFileInfo>
#include <QStandardPaths>
#include <QDesktopServices>
#include <QUrl>
#include <QProgressDialog>

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
    // ROS_PRINT("TF Struct---");
    // ROS_PRINT("\r\n%s", tf_tree.toString().c_str());

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
    // define tf_tree file name and path
    QString tempDirPath = QStandardPaths::writableLocation(QStandardPaths::TempLocation);
    QString tempFileName = "tf_tree.pdf";

    QFileInfo fileInfo(tempDirPath + "/" + tempFileName);
    QString pathWithoutExtension = fileInfo.path() + "/" + fileInfo.baseName();
    QString pathWithExtension = fileInfo.filePath();

    // 创建进度对话框
    QProgressDialog progressDialog(this);
    progressDialog.setLabelText("Running script...");
    progressDialog.setCancelButtonText("Cancel");
    progressDialog.setRange(0, 5000); // 大概5秒跑完

    // 设置对话框为模态
    progressDialog.setModal(true);
    // 显示对话框，并等待用户操作
    progressDialog.show();

    // run script "ros2 run tf2_tools view_frames -o /tmp/tf_treeby" by QProcess
    QProcess *process = new QProcess(this);
    QString command = "ros2";
    QStringList arguments;
    arguments << "run"
              << "tf2_tools"
              << "view_frames"
              << "-o" << pathWithoutExtension;

    process->start(command, arguments);
    ROS_PRINT("Process Start: %s %s", command.toStdString().c_str(), arguments.join(" ").toStdString().c_str());

    // 等待进程处理完成
    int k_interval = 100;
    int process_time = 0;
    while (!process->waitForFinished(k_interval))
    {
        if (progressDialog.wasCanceled())
        {
            process->terminate();
            ROS_PRINT("Process canceled.");
            return;
        }

        // 更新進度條
        process_time+=k_interval;
        if(process_time >= progressDialog.maximum())
            process_time = progressDialog.maximum()-1;
        progressDialog.setValue(process_time);
        QCoreApplication::processEvents();
    }
    // 关闭进度对话框
    progressDialog.close();

    // open pdf file
    ROS_PRINT("Open File: %s", pathWithExtension.toStdString().c_str());
    QUrl fileUrl = QUrl::fromLocalFile(pathWithExtension);
    if (!QDesktopServices::openUrl(fileUrl))
    {
        ROS_PRINT("Failed to open file.");
        return;
    }
}
