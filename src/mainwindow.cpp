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
#include <QDateTime>
#include <QStringLiteral>

#define ROS_PRINT(...) RCLCPP_INFO(this->node_->get_logger(), __VA_ARGS__)

#define RAD2DEG 180.0/M_PI

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

    QObject::connect(&this->tf_lookup_timer_,
                     &QTimer::timeout,
                     this,
                     &MainWindow::on_tf_lookup_timer_tick);
    this->tf_lookup_timer_.start(30);
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

void MainWindow::updateComboBox(QComboBox *cbox, std::vector<std::string> values)
{
    // 清空现有的项
    cbox->clear();

    for (const std::string&  v : values)
    {
        QString qValue = QString::fromStdString(v);
        cbox->addItem(qValue);
    }
}

void MainWindow::expandTreeNextLevel(QTreeWidgetItem *item, bool next_stop)
{
    // Check if the item is a null pointer, return if true
    if (!item)
        return;

    // If next_stop is set to true, it means the next level has been expanded and should stop expanding
    if(next_stop)
        return;

    // If the current item is not expanded yet, expand it and set next_stop to true to stop expanding the next level
    if (!item->isExpanded())
    {
        item->setExpanded(true);
        next_stop = true;
    }

    // Recursively expand child items
    for (int i = 0; i < item->childCount(); ++i)
    {
        QTreeWidgetItem *childItem = item->child(i);
        expandTreeNextLevel(childItem, next_stop);
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

    auto tf_names = tf_tree.getAllTreeValues();

    this->updateComboBox(ui->cbox_reference, tf_names);
    this->updateComboBox(ui->cbox_target, tf_names);

}


void MainWindow::on_btn_expand_all_clicked()
{
    // ui->treeWidget->expandAll();
    // Expand top level items
    for (int i = 0; i < ui->treeWidget->topLevelItemCount(); ++i)
    {
        QTreeWidgetItem *item = ui->treeWidget->topLevelItem(i);
        if (item)
        {
            // Call the function to expand the tree structure
            this->expandTreeNextLevel(item);
        }
    }
}

void MainWindow::on_btn_collapse_all_clicked()
{
    ui->treeWidget->collapseAll();
}

void MainWindow::on_btn_view_frames_clicked()
{
    // Define tf_tree file name and path
    QString tempDirPath = QStandardPaths::writableLocation(QStandardPaths::TempLocation);
    QString tempFileName = "tf_tree.pdf";

    QFileInfo fileInfo(tempDirPath + "/" + tempFileName);
    QString pathWithoutExtension = fileInfo.path() + "/" + fileInfo.baseName();
    QString pathWithExtension = fileInfo.filePath();

    // Create a progress dialog
    QProgressDialog progressDialog(this);
    progressDialog.setLabelText("Running script...");
    progressDialog.setCancelButtonText("Cancel");
    progressDialog.setRange(0, 5000); // Approximately 5 seconds to complete

    // Set the dialog to be modal
    progressDialog.setModal(true);
    // Show the dialog and wait for user interaction
    progressDialog.show();

    // Run the script "ros2 run tf2_tools view_frames -o /tmp/tf_treeby" using QProcess
    QProcess *process = new QProcess(this);
    QString command = "ros2";
    QStringList arguments;
    arguments << "run"
              << "tf2_tools"
              << "view_frames"
              << "-o" << pathWithoutExtension;

    process->start(command, arguments);
    ROS_PRINT("Process Start: %s %s", command.toStdString().c_str(), arguments.join(" ").toStdString().c_str());

    // Wait for the process to finish
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

        // Update the progress bar
        process_time += k_interval;
        if (process_time >= progressDialog.maximum())
            process_time = progressDialog.maximum() - 1;
        progressDialog.setValue(process_time);
        QCoreApplication::processEvents();
    }
    // Close the progress dialog
    progressDialog.close();

    // Open the PDF file
    ROS_PRINT("Open File: %s", pathWithExtension.toStdString().c_str());
    QUrl fileUrl = QUrl::fromLocalFile(pathWithExtension);
    if (!QDesktopServices::openUrl(fileUrl))
    {
        ROS_PRINT("Failed to open file.");
        return;
    }
}


void MainWindow::on_btn_reference_clicked()
{
    // Get the currently selected item in the QTreeWidget
    QTreeWidgetItem *selectedItem = ui->treeWidget->currentItem();
    if (selectedItem) {
        // Get the text of the selected item
        QString selectedText = selectedItem->text(0);

        // Find the corresponding item in the QComboBox and set it as the current item
        int index = ui->cbox_reference->findText(selectedText);
        if (index != -1)
        {
            ui->cbox_reference->setCurrentIndex(index);
        }
    }
}

void MainWindow::on_btn_target_clicked()
{
    // Get the currently selected item in the QTreeWidget
    QTreeWidgetItem *selectedItem = ui->treeWidget->currentItem();
    if (selectedItem) {
        // Get the text of the selected item
        QString selectedText = selectedItem->text(0);

        // Find the corresponding item in the QComboBox and set it as the current item
        int index = ui->cbox_target->findText(selectedText);
        if (index != -1)
        {
            ui->cbox_target->setCurrentIndex(index);
        }
    }
}

QString rostimeToString(const rclcpp::Time& time)
{
    QDateTime dateTime = QDateTime::fromMSecsSinceEpoch(time.nanoseconds()/1000000);
    QString formattedDateTime = dateTime.toString("yyyy-MM-dd hh:mm:ss.zzz");
    return formattedDateTime;
}

void MainWindow::on_tf_lookup_timer_tick()
{
    // ROS_PRINT("on_tf_lookup_timer_tick");
    std::string reference_tf_name = ui->cbox_reference->currentText().toStdString();
    std::string target_tf_name = ui->cbox_target->currentText().toStdString();
    if(reference_tf_name == "" || target_tf_name == "")
        return;

    try
    {
        geometry_msgs::msg::TransformStamped transform_stamped = this->tf_buffer_->lookupTransform(reference_tf_name, target_tf_name, tf2::TimePointZero);

        QString str_timestamp = QString("Timestamp: %1 \r\n").arg(rostimeToString(this->node_->get_clock()->now()));
        QString str_position = QString("Position:\r\n (x,y,z): (%1, %2, %3)\r\n")
                .arg(transform_stamped.transform.translation.x, 0, 'f', 6)
                .arg(transform_stamped.transform.translation.y, 0, 'f', 6)
                .arg(transform_stamped.transform.translation.z, 0, 'f', 6);

        tf2::Quaternion tf_quaternion;
        tf2::fromMsg(transform_stamped.transform.rotation, tf_quaternion);

        QString str_quaternion = QString("(x,y,z,w): (%1, %2, %3, %4)")
                .arg(QString::number(tf_quaternion.getX(), 'f', 6))
                .arg(QString::number(tf_quaternion.getY(), 'f', 6))
                .arg(QString::number(tf_quaternion.getZ(), 'f', 6))
                .arg(QString::number(tf_quaternion.getW(), 'f', 6));

        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
        QString str_rpy = QString("(roll,pitch,yaw): (%1, %2, %3)")
                .arg(QString::number(roll*RAD2DEG, 'f', 2))
                .arg(QString::number(pitch*RAD2DEG, 'f', 2))
                .arg(QString::number(yaw*RAD2DEG, 'f', 2));

        QString str_rotation = QString("Rotation:\r\n"
                                       " %1\r\n"
                                       " %2\r\n")
                .arg(str_quaternion)
                .arg(str_rpy);


        ui->tbox_tf_info->setText(str_timestamp + str_position + str_rotation);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "TF Exception：%s", ex.what());
    }

}

