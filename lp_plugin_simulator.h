#ifndef LP_PLUGIN_SIMULATOR_H
#define LP_PLUGIN_SIMULATOR_H

#include "LP_Plugin_Simulator_global.h"

#include "plugin/lp_actionplugin.h"

#include <QObject>
#include <QOpenGLBuffer>
#include <QCheckBox>
#include <QVector2D>
#include <QVector3D>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/opencv.hpp"

#include <math.h>
#include <QProcess>
#include <QFile>
#include <QTextStream>
#include <QThread>
#include <QPushButton>

/**
 * @brief BulletPhysics Headers
 */
#include <SoftDemo/SoftDemo.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <Bullet3Dynamics/b3CpuRigidBodyPipeline.h>
#include <CommonInterfaces/CommonRigidBodyBase.h>

class QLabel;
class LP_ObjectImpl;
class QOpenGLShaderProgram;
class btSoftRigidDynamicsWorld;


/**
 * @brief The LP_Plugin_Simulator class
 */

#define LP_Plugin_Simulator_iid "cpii.rp5.SmartFashion.LP_Plugin_Simulator/0.1"

class LP_Plugin_Simulator : public LP_ActionPlugin
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID LP_Plugin_Simulator_iid)
    Q_INTERFACES(LP_ActionPlugin)

public:
    virtual ~LP_Plugin_Simulator();

        // LP_Functional interface
        QWidget *DockUi() override;
        bool Run() override;
        bool eventFilter(QObject *watched, QEvent *event) override;

        // LP_ActionPlugin interface
        QString MenuName();
        QAction *Trigger();

        bool m_cutting;

        btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
        btSoftBodyWorldInfo m_softBodyWorldInfo;
        btCollisionDispatcher* m_dispatcher;
        btConstraintSolver* m_solver;
        btDefaultCollisionConfiguration* m_collisionConfiguration;
        btBroadphaseInterface* m_broadphase;
        btDiscreteDynamicsWorld* m_dynamicsWorld;
        std::vector<btSoftBody*> m_SoftBodies;

        void initPhysics();
        void exitPhysics();

        virtual const btSoftRigidDynamicsWorld* getSoftDynamicsWorld() const;
        virtual btSoftRigidDynamicsWorld* getSoftDynamicsWorld();

signals:


        // LP_Functional interface
public slots:

        void FunctionalRender_L(QOpenGLContext *ctx, QSurface *surf, QOpenGLFramebufferObject *fbo, const LP_RendererCam &cam, const QVariant &options) override;
        void FunctionalRender_R(QOpenGLContext *ctx, QSurface *surf, QOpenGLFramebufferObject *fbo, const LP_RendererCam &cam, const QVariant &options) override;

private:
        bool mInitialized_L = false;
        bool mInitialized_R = false;
        bool gStarted = false;
        bool gSavedMesh = false;
        std::weak_ptr<LP_ObjectImpl> mObject;
        std::shared_ptr<QWidget> mWidget;
        QLabel *mObjectid = nullptr;
        QLabel *mLabel = nullptr;
        QPushButton *mButton = nullptr;
        QOpenGLShaderProgram *mProgram_L = nullptr,
                             *mProgram_R = nullptr;

        /**
         * @brief initializeGL initalize any OpenGL resource
         */
        void initializeGL_L();
        void initializeGL_R();

};


#endif // LP_PLUGIN_SIMULATOR_H
