#include "tf_preview.h"
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

TfPreview::TfPreview(QWidget *parent) : QWidget(parent)
{
    setMinimumSize(300, 230);
    setAccessibleName("3D frame preview");
    setToolTip("Drag to rotate; use the wheel to zoom.");
    connect(&timer_, &QTimer::timeout, this, &TfPreview::tick);
    clock_.start();
    timer_.start(16);
}
void TfPreview::setPose(const QVector3D &position, const QQuaternion &orientation)
{
    position_ = position;
    orientation_ = orientation;
    if (autoFrame_) {
        targetCenter_ = position_ * 0.5f;
        targetDistance_ = std::max(2.5, static_cast<double>(position_.length()) * 1.8 + 2.0);
    }
    update();
}
void TfPreview::setFrameNames(const QString &parent, const QString &child)
{
    parentName_ = parent; childName_ = child; update();
}
void TfPreview::resumeOrbit() { orbit_ = true; }
void TfPreview::resetView()
{
    orbit_ = true; autoFrame_ = true;
    targetPitch_ = 0.45;
    targetCenter_ = position_ * 0.5f;
    targetDistance_ = std::max(2.5, static_cast<double>(position_.length()) * 1.8 + 2.0);
}
void TfPreview::tick()
{
    double dt = std::min(0.1, clock_.restart() / 1000.0);
    if (orbit_) targetYaw_ += dt * (2.0 * M_PI / 30.0);
    const float alpha = static_cast<float>(1.0 - std::exp(-dt * 8.0));
    yaw_ += (targetYaw_ - yaw_) * alpha;
    pitch_ += (targetPitch_ - pitch_) * alpha;
    distance_ += (targetDistance_ - distance_) * alpha;
    center_ += (targetCenter_ - center_) * alpha;
    update();
}
void TfPreview::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) { orbit_ = false; autoFrame_ = false; lastMouse_ = event->pos(); event->accept(); }
}
void TfPreview::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::LeftButton) {
        QPoint delta = event->pos() - lastMouse_; lastMouse_ = event->pos();
        targetYaw_ -= delta.x() * 0.01;
        targetPitch_ = std::clamp(targetPitch_ + delta.y() * 0.01, -1.45, 1.45);
        event->accept();
    }
}
void TfPreview::wheelEvent(QWheelEvent *event)
{
    orbit_ = false; autoFrame_ = false;
    targetDistance_ = std::clamp(targetDistance_ * std::exp(-event->angleDelta().y() / 1200.0), 0.05, 1e12);
    event->accept();
}
void TfPreview::paintEvent(QPaintEvent *)
{
    QPainter p(this); p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(28, 35, 46));
    QVector3D forward(static_cast<float>(std::cos(pitch_) * std::cos(yaw_)), static_cast<float>(std::cos(pitch_) * std::sin(yaw_)), static_cast<float>(std::sin(pitch_)));
    QVector3D right(-forward.y(), forward.x(), 0);
    QVector3D up = QVector3D::crossProduct(forward, right).normalized();
    const double scale = std::min(width(), height()) * 0.75 / distance_;
    auto project = [&](const QVector3D &v) {
        QVector3D d = v - center_;
        return QPointF(width() * 0.5 + QVector3D::dotProduct(d, right) * scale,
                       height() * 0.5 - QVector3D::dotProduct(d, up) * scale);
    };
    auto drawFrame = [&](const QVector3D &origin, const QQuaternion &rotation, const QString &name) {
        const QVector3D axes[] = {QVector3D(1,0,0), QVector3D(0,1,0), QVector3D(0,0,1)};
        const QColor colors[] = {QColor(255,90,90), QColor(105,225,130), QColor(105,165,255)};
        const char *labels[] = {"X", "Y", "Z"};
        const double length = std::max(0.4, static_cast<double>(position_.length()) * 0.16);
        for (int i=0;i<3;++i) {
            QPointF start = project(origin), end = project(origin + rotation.rotatedVector(axes[i]) * static_cast<float>(length));
            p.setPen(QPen(colors[i], 2.5)); p.drawLine(start, end); p.drawText(end + QPointF(4,-4), labels[i]);
        }
        p.setPen(QColor(235,240,245)); p.drawText(project(origin) + QPointF(5,16), name.isEmpty() ? QStringLiteral("frame") : name);
    };
    p.setPen(QPen(QColor(150,160,174), 1, Qt::DashLine)); p.drawLine(project(QVector3D()), project(position_));
    drawFrame(QVector3D(), QQuaternion(), parentName_);
    drawFrame(position_, orientation_, childName_);
}
