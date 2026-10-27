#include "static_tf_panel.h"
#include "tf_preview.h"
#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QClipboard>
#include <QComboBox>
#include <QCompleter>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QSettings>
#include <QStandardPaths>
#include <QVBoxLayout>
#include <QSignalBlocker>
#include <tf2/LinearMath/Quaternion.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace {
QString settingsFile() { return QStandardPaths::writableLocation(QStandardPaths::GenericConfigLocation) + "/arc_tf_viewer/settings.ini"; }
QString num(double v) { return QString::number(v == 0 ? 0 : v, 'g', 16); }
}

StaticTfPanel::StaticTfPanel(rclcpp::Node::SharedPtr node, tf2_ros::Buffer &buffer, QWidget *parent)
    : QDialog(parent), node_(std::move(node)), buffer_(buffer)
{
    setWindowTitle("Static TF editor");
    setModal(false);
    resize(950, 700);
    auto *layout = new QVBoxLayout(this);
    auto *top = new QHBoxLayout;
    auto *editor = new QVBoxLayout;
    auto *frames = new QGroupBox("Frames");
    auto *form = new QFormLayout(frames);
    parentFrame_ = new QComboBox; parentFrame_->setObjectName("parentFrame"); parentFrame_->setEditable(true);
    childFrame_ = new QComboBox; childFrame_->setObjectName("childFrame"); childFrame_->setEditable(true);
    for (auto *combo : {parentFrame_, childFrame_}) {
        combo->setInsertPolicy(QComboBox::NoInsert);
        combo->completer()->setFilterMode(Qt::MatchContains);
        combo->completer()->setCaseSensitivity(Qt::CaseInsensitive);
    }
    form->addRow("Parent frame", parentFrame_); form->addRow("Child frame", childFrame_);
    auto *refresh = new QPushButton("Refresh frames"); refresh->setObjectName("refreshFrames"); form->addRow(refresh);
    editor->addWidget(frames);
    auto *pose = new QGroupBox("Pose");
    auto *poseLayout = new QVBoxLayout(pose);
    auto *stepRow = new QHBoxLayout; stepRow->addWidget(new QLabel("Step"));
    auto *group = new QButtonGroup(this);
    const double steps[] = {10, 1, 0.1, 0.01};
    for (double step : steps) {
        auto *radio = new QRadioButton(QString("×%1").arg(num(step)));
        radio->setObjectName("step_" + num(step)); group->addButton(radio); stepRow->addWidget(radio);
        if (step == 0.01) radio->setChecked(true);
        connect(radio, &QRadioButton::toggled, this, [this, step](bool on) { if (on) updateStep(step); });
    }
    stepRow->addStretch(); poseLayout->addLayout(stepRow);
    auto *unitRow = new QHBoxLayout;
    unitRow->addWidget(new QLabel("XYZ unit")); xyzUnit_ = new QPushButton("m"); xyzUnit_->setObjectName("xyzUnit"); unitRow->addWidget(xyzUnit_);
    unitRow->addWidget(new QLabel("RPY unit")); rpyUnit_ = new QPushButton("rad"); rpyUnit_->setObjectName("rpyUnit"); unitRow->addWidget(rpyUnit_); unitRow->addStretch(); poseLayout->addLayout(unitRow);
    auto *grid = new QGridLayout;
    const char *names[] = {"X", "Y", "Z", "Roll", "Pitch", "Yaw"};
    for (int i=0; i<6; ++i) {
        auto *label = new QLabel(names[i]); auto *minus = new QPushButton("−"); auto *plus = new QPushButton("+");
        auto *spin = new QDoubleSpinBox; axes_[i] = spin; spin->setObjectName(QString("axis%1").arg(names[i]));
        spin->setDecimals(12); spin->setRange(-std::numeric_limits<double>::max() / 1000.0, std::numeric_limits<double>::max() / 1000.0); spin->setSingleStep(step_); spin->setKeyboardTracking(true);
        spin->setButtonSymbols(QAbstractSpinBox::NoButtons); spin->setAccessibleName(names[i]);
        minus->setAccessibleName(QString("Decrease %1").arg(names[i])); plus->setAccessibleName(QString("Increase %1").arg(names[i]));
        minus->setObjectName(QString("decrease%1").arg(names[i])); plus->setObjectName(QString("increase%1").arg(names[i]));
        for (auto *b : {minus, plus}) { b->setAutoRepeat(true); b->setAutoRepeatDelay(400); b->setAutoRepeatInterval(60); b->setFixedWidth(34); }
        grid->addWidget(label,i,0); grid->addWidget(minus,i,1); grid->addWidget(spin,i,2); grid->addWidget(plus,i,3);
        connect(minus, &QPushButton::clicked, spin, &QDoubleSpinBox::stepDown);
        connect(plus, &QPushButton::clicked, spin, &QDoubleSpinBox::stepUp);
        connect(spin, &QDoubleSpinBox::editingFinished, this, [this] { if (enable_->isChecked()) publishTransform(); });
        connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this,i](double v) {
            if (updating_) return;
            values_[i] = v / displayFactor(i); updateOutput(); if (enable_->isChecked()) publishTransform();
        });
    }
    poseLayout->addLayout(grid); editor->addWidget(pose); top->addLayout(editor, 1);
    auto *previewColumn = new QVBoxLayout; auto *previewTitle = new QLabel("Frame preview · X red / Y green / Z blue");
    previewColumn->addWidget(previewTitle); preview_ = new TfPreview; preview_->setObjectName("tfPreview"); previewColumn->addWidget(preview_, 1);
    auto *camera = new QHBoxLayout; auto *orbit = new QPushButton("Auto orbit"); orbit->setObjectName("autoOrbit"); auto *reset = new QPushButton("Reset view"); reset->setObjectName("resetView"); camera->addWidget(orbit); camera->addWidget(reset); previewColumn->addLayout(camera); top->addLayout(previewColumn, 1);
    layout->addLayout(top, 1);
    auto *output = new QGroupBox("URDF preview"); auto *outputLayout = new QVBoxLayout(output); auto *jointRow = new QHBoxLayout;
    jointRow->addWidget(new QLabel("Joint name")); jointName_ = new QLineEdit; jointName_->setObjectName("jointName"); jointRow->addWidget(jointName_);
    includeLink_ = new QCheckBox("Include child link"); includeLink_->setObjectName("includeChildLink"); includeLink_->setChecked(true); jointRow->addWidget(includeLink_); outputLayout->addLayout(jointRow);
    urdfPreview_ = new QPlainTextEdit; urdfPreview_->setObjectName("urdfPreview"); urdfPreview_->setReadOnly(true); urdfPreview_->setMaximumHeight(145); outputLayout->addWidget(urdfPreview_);
    auto *copy = new QPushButton("Copy URDF"); copy->setObjectName("copyUrdf"); outputLayout->addWidget(copy, 0, Qt::AlignRight); layout->addWidget(output);
    auto *bottom = new QHBoxLayout; enable_ = new QCheckBox("Enable static TF"); enable_->setObjectName("enableTf"); bottom->addWidget(enable_);
    status_ = new QLabel("Disabled. Published static TFs may remain in other nodes' caches."); status_->setObjectName("status"); status_->setWordWrap(true); bottom->addWidget(status_, 1); layout->addLayout(bottom);
    connect(refresh, &QPushButton::clicked, this, &StaticTfPanel::refreshFrames);
    connect(xyzUnit_, &QPushButton::clicked, this, [this] { xyzUnitIndex_ = (xyzUnitIndex_ + 1) % 3; updateUnits(); });
    connect(rpyUnit_, &QPushButton::clicked, this, [this] { rpyUnitIndex_ = (rpyUnitIndex_ + 1) % 2; updateUnits(); });
    connect(orbit, &QPushButton::clicked, preview_, &TfPreview::resumeOrbit);
    connect(reset, &QPushButton::clicked, preview_, &TfPreview::resetView);
    connect(copy, &QPushButton::clicked, this, [this] { QApplication::clipboard()->setText(urdfPreview_->toPlainText()); });
    connect(jointName_, &QLineEdit::textEdited, this, [this] { jointEdited_ = true; });
    connect(jointName_, &QLineEdit::textChanged, this, [this] { updateOutput(); });
    connect(includeLink_, &QCheckBox::toggled, this, [this] { updateOutput(); });
    connect(includeLink_, &QCheckBox::clicked, this, [this] { includeLinkEdited_ = true; });
    auto frameChanged = [this] {
        if (loading_) return;
        if (!jointEdited_) jointName_->setText(parentFrame_->currentText() + "_to_" + childFrame_->currentText() + "_joint");
        updateOutput();
    };
    connect(parentFrame_, &QComboBox::editTextChanged, this, frameChanged);
    connect(childFrame_, &QComboBox::editTextChanged, this, [this, frameChanged] {
        if (!loading_) { includeLinkEdited_ = false; auto frames = frameParents(); includeLink_->setChecked(frames.count(childFrame_->currentText()) == 0); }
        frameChanged();
    });
    connect(enable_, &QCheckBox::toggled, this, &StaticTfPanel::setEnabledPublishing);
    loadSettings(); refreshFrames();
    includeLink_->setChecked(frameParents().count(childFrame_->currentText()) == 0);
    updateUnits(); updateOutput();
}
StaticTfPanel::~StaticTfPanel() { saveSettings(); }

double StaticTfPanel::displayFactor(int index) const
{
    if (index < 3) { const double factors[] = {1, 100, 1000}; return factors[xyzUnitIndex_]; }
    return rpyUnitIndex_ == 0 ? 1.0 : 180.0 / M_PI;
}
void StaticTfPanel::updateUnits()
{
    updating_ = true;
    xyzUnit_->setText(QStringList{"m","cm","mm"}[xyzUnitIndex_]);
    rpyUnit_->setText(rpyUnitIndex_ == 0 ? "rad" : "deg");
    for (int i=0;i<6;++i) {
        const double limit = (std::numeric_limits<double>::max() / 1000.0) * displayFactor(i);
        axes_[i]->setRange(-limit, limit);
        axes_[i]->setValue(values_[i] * displayFactor(i));
    }
    updating_ = false; updateOutput();
}
void StaticTfPanel::updateStep(double step) { step_ = step; for (auto *axis : axes_) axis->setSingleStep(step); }
std::map<QString, QString> StaticTfPanel::frameParents() const
{
    std::map<QString, QString> result;
    try {
        YAML::Node frames = YAML::Load(buffer_.allFramesAsYAML());
        if (!frames.IsMap()) return result;
        for (const auto &entry : frames) {
            QString child = QString::fromStdString(entry.first.as<std::string>());
            QString parent = QString::fromStdString(entry.second["parent"].as<std::string>());
            result[child] = parent;
            if (!result.count(parent)) result[parent] = QString();
        }
    } catch (const YAML::Exception &ex) { RCLCPP_WARN(node_->get_logger(), "Cannot read TF frames: %s", ex.what()); }
    return result;
}
void StaticTfPanel::refreshFrames()
{
    auto frames = frameParents();
    for (auto *combo : {parentFrame_, childFrame_}) {
        QString selected = combo->currentText(); QSignalBlocker block(combo); combo->clear();
        for (const auto &entry : frames) combo->addItem(entry.first);
        combo->setEditText(selected);
    }
    if (!includeLinkEdited_) includeLink_->setChecked(frames.count(childFrame_->currentText()) == 0);
    updateOutput();
}
QString StaticTfPanel::validateFrames() const
{
    const QString parent = parentFrame_->currentText().trimmed(), child = childFrame_->currentText().trimmed();
    if (!inputsValid()) return "Complete all six numeric fields before enabling.";
    if (parent.isEmpty() || child.isEmpty()) return "Parent and child are required.";
    if (parent.startsWith('/') || child.startsWith('/') || parent.contains(' ') || child.contains(' ')) return "Frame names cannot start with '/' or contain spaces.";
    if (parent == child) return "Parent and child must differ.";
    auto frames = frameParents();
    if (!frames.count(parent)) return "Parent must be an existing frame. Refresh frames and try again.";
    auto it = frames.find(child);
    if (it == frames.end()) return {};
    auto own = published_.find(child);
    if (!it->second.isEmpty() && !(own != published_.end() && own->second == parent && it->second == parent)) return "Child already has a parent.";
    if (own != published_.end() && own->second == parent && it->second == parent) return {};
    std::set<QString> seen;
    QString cursor = parent;
    while (!cursor.isEmpty() && seen.insert(cursor).second) {
        if (cursor == child) return "Connecting these frames would create a cycle.";
        auto found = frames.find(cursor); cursor = found == frames.end() ? QString() : found->second;
    }
    if (!seen.insert(cursor).second && !cursor.isEmpty()) return "The existing TF graph contains a cycle.";
    return {};
}
void StaticTfPanel::setEnabledPublishing(bool enabled)
{
    if (enabled) {
        QString error = validateFrames();
        if (!error.isEmpty()) { QSignalBlocker block(enable_); enable_->setChecked(false); status_->setText(error); return; }
        broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
        parentFrame_->setEnabled(false); childFrame_->setEnabled(false);
        status_->setText("Enabled · publishing static TF on changes. Disable cannot retract cached transforms.");
        publishTransform();
    } else {
        broadcaster_.reset(); parentFrame_->setEnabled(true); childFrame_->setEnabled(true);
        status_->setText("Disabled. Published static TFs may remain in other nodes' caches.");
    }
}
bool StaticTfPanel::inputsValid() const
{
    for (auto *axis : axes_) {
        const auto *input = axis->findChild<QLineEdit *>();
        if (!input || !input->hasAcceptableInput() || !std::isfinite(axis->value())) return false;
    }
    return true;
}
void StaticTfPanel::publishTransform()
{
    if (!broadcaster_ || !enable_->isChecked() || !inputsValid()) return;
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = node_->now(); msg.header.frame_id = parentFrame_->currentText().trimmed().toStdString(); msg.child_frame_id = childFrame_->currentText().trimmed().toStdString();
    msg.transform.translation.x = values_[0]; msg.transform.translation.y = values_[1]; msg.transform.translation.z = values_[2];
    tf2::Quaternion q; q.setRPY(values_[3], values_[4], values_[5]);
    msg.transform.rotation.x=q.x(); msg.transform.rotation.y=q.y(); msg.transform.rotation.z=q.z(); msg.transform.rotation.w=q.w();
    broadcaster_->sendTransform(msg); published_[childFrame_->currentText().trimmed()] = parentFrame_->currentText().trimmed();
}
void StaticTfPanel::updateOutput()
{
    QString parent = parentFrame_->currentText().trimmed(), child = childFrame_->currentText().trimmed();
    QString xml;
    if (includeLink_->isChecked()) xml += QString("<link name=\"%1\"/>\n").arg(child.toHtmlEscaped());
    xml += QString("<joint name=\"%1\" type=\"fixed\">\n  <parent link=\"%2\"/>\n  <child link=\"%3\"/>\n  <origin xyz=\"%4 %5 %6\" rpy=\"%7 %8 %9\"/>\n</joint>")
        .arg(jointName_->text().toHtmlEscaped(), parent.toHtmlEscaped(), child.toHtmlEscaped(), num(values_[0]), num(values_[1]), num(values_[2]), num(values_[3]), num(values_[4]), num(values_[5]));
    urdfPreview_->setPlainText(xml);
    tf2::Quaternion q; q.setRPY(values_[3],values_[4],values_[5]);
    // Keep the rendering coordinates finite even for very large valid TF values.
    const double largest = std::max({std::abs(values_[0]), std::abs(values_[1]), std::abs(values_[2])});
    const double previewScale = largest > 1000000.0 ? 1000000.0 / largest : 1.0;
    preview_->setPose(QVector3D(values_[0] * previewScale, values_[1] * previewScale, values_[2] * previewScale),
                      QQuaternion(q.w(),q.x(),q.y(),q.z()));
    preview_->setFrameNames(parent, child);
}
void StaticTfPanel::saveSettings()
{
    QSettings settings(settingsFile(), QSettings::IniFormat);
    settings.setValue("parent", parentFrame_->currentText()); settings.setValue("child", childFrame_->currentText());
    for (int i=0;i<6;++i) settings.setValue(QString("axis%1").arg(i), values_[i]);
    settings.setValue("xyzUnit", xyzUnitIndex_); settings.setValue("rpyUnit", rpyUnitIndex_); settings.setValue("step",step_);
}
void StaticTfPanel::loadSettings()
{
    loading_ = true; QSettings settings(settingsFile(), QSettings::IniFormat);
    parentFrame_->setEditText(settings.value("parent").toString()); childFrame_->setEditText(settings.value("child").toString());
    for (int i=0;i<6;++i) values_[i] = settings.value(QString("axis%1").arg(i),0).toDouble();
    xyzUnitIndex_ = std::clamp(settings.value("xyzUnit",0).toInt(),0,2); rpyUnitIndex_ = std::clamp(settings.value("rpyUnit",0).toInt(),0,1);
    double savedStep = settings.value("step",0.01).toDouble();
    if (auto *radio = findChild<QRadioButton *>("step_" + num(savedStep))) radio->setChecked(true);
    loading_ = false;
    jointName_->setText(parentFrame_->currentText() + "_to_" + childFrame_->currentText() + "_joint");
}
