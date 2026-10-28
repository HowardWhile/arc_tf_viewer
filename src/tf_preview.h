#ifndef TF_PREVIEW_H
#define TF_PREVIEW_H

#include <QWidget>
#include <QVector3D>
#include <QQuaternion>
#include <QTimer>
#include <QElapsedTimer>

class TfPreview : public QWidget
{
    Q_OBJECT
public:
    explicit TfPreview(QWidget *parent = nullptr);
    void setPose(const QVector3D &position, const QQuaternion &orientation);
    void setFrameNames(const QString &parent, const QString &child);
    void resumeOrbit();
    void resetView();
protected:
    void paintEvent(QPaintEvent *) override;
    void mousePressEvent(QMouseEvent *) override;
    void mouseMoveEvent(QMouseEvent *) override;
    void wheelEvent(QWheelEvent *) override;
private:
    void tick();
    QVector3D position_, center_, targetCenter_;
    QQuaternion orientation_;
    QString parentName_, childName_;
    QTimer timer_;
    QElapsedTimer clock_;
    QPoint lastMouse_;
    double yaw_ = 0.8, targetYaw_ = 0.8;
    double pitch_ = 0.45, targetPitch_ = 0.45;
    double distance_ = 3.0, targetDistance_ = 3.0;
    bool orbit_ = true, autoFrame_ = true;
};
#endif
