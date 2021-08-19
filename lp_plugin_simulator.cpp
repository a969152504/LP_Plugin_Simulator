#include "lp_plugin_simulator.h"

#include "lp_renderercam.h"
#include "lp_openmesh.h"
#include "renderer/lp_glselector.h"
#include "renderer/lp_glrenderer.h"

#include <math.h>

#include <QVBoxLayout>
#include <QMouseEvent>
#include <QOpenGLContext>
#include <QOpenGLShaderProgram>
#include <QOpenGLExtraFunctions>
#include <QLabel>
#include <QMatrix4x4>
#include <QPushButton>
#include <QtConcurrent/QtConcurrent>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>


const int maxProxies = 32766;
int gNumofTri;
QFuture<void> gFuture;
bool gRun = true, gInitPhysics = false;
btAlignedObjectArray<btScalar> gPoints;
btAlignedObjectArray<int> gIndices;
const int NUM_VERTS_X = 30;
const int NUM_VERTS_Y = 30;
const int totalVerts = NUM_VERTS_X * NUM_VERTS_Y;
const int totalTriangles = 2 * (NUM_VERTS_X - 1) * (NUM_VERTS_Y - 1);
static btVector3* gGroundVertices = 0;
static int* gGroundIndices = 0;
const float TRIANGLE_SIZE = 8.f;
static float waveheight = 5.f;
#define CUBE_HALF_EXTENTS 1.5


class Sleeper : public QThread
{
public:
    static void usleep(unsigned long usecs){QThread::usleep(usecs);}
    static void msleep(unsigned long msecs){QThread::msleep(msecs);}
    static void sleep(unsigned long secs){QThread::sleep(secs);}
};

const btSoftRigidDynamicsWorld* LP_Plugin_Simulator::getSoftDynamicsWorld() const
{
    ///just make it a btSoftRigidDynamicsWorld please
    ///or we will add type checking
    return (btSoftRigidDynamicsWorld*)m_dynamicsWorld;
}

btSoftRigidDynamicsWorld* LP_Plugin_Simulator::getSoftDynamicsWorld()
{
    ///just make it a btSoftRigidDynamicsWorld please
    ///or we will add type checking
    return (btSoftRigidDynamicsWorld*)m_dynamicsWorld;
}

static void Ctor_RbUpStack(LP_Plugin_Simulator* pdemo, int count)
{
    btCompoundShape* cylinderCompound = new btCompoundShape;
    btCollisionShape* cylinderShape = new btCylinderShapeX(btVector3(4, 1, 1));
    btCollisionShape* boxShape = new btBoxShape(btVector3(4, 1, 1));
    btTransform localTransform;
    localTransform.setIdentity();
    cylinderCompound->addChildShape(localTransform, boxShape);
    btQuaternion orn(SIMD_HALF_PI, 0, 0);
    localTransform.setRotation(orn);
    cylinderCompound->addChildShape(localTransform, cylinderShape);

    for (int i = 0; i < count; ++i)
    {
        btTransform startTransform;
        startTransform.setIdentity();
        startTransform.setOrigin(btVector3(0, 2 + 6 * i, 0));
    }
}

static void Init_Cloth(LP_Plugin_Simulator* pdemo)
{
    //TRACEDEMO
    pdemo->initPhysics();
    gInitPhysics = true;

    qDebug() << "Done Init_Phy";

//    const btScalar s = 10;
//    btSoftBody* psb = btSoftBodyHelpers::CreatePatch(pdemo->m_softBodyWorldInfo, btVector3(-s, 0, -s),
//                                                     btVector3(+s, 0, -s),
//                                                     btVector3(-s, 0, +s),
//                                                     btVector3(+s, 0, +s),
//                                                     31, 31,
//                                                     //		31,31,
//                                                     1 + 2 + 4 + 8, true);
//    psb->getCollisionShape()->setMargin(0.5);
//    btSoftBody::Material* pm = psb->appendMaterial();
//    pm->m_kLST = 0.4;
//    pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
//    psb->generateBendingConstraints(2, pm);
//    psb->setTotalMass(150);
//    pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

//    pdemo->m_SoftBodies.push_back(psb);

//    Ctor_RbUpStack(pdemo, 10);
//    pdemo->m_cutting = true;

    btSoftBody* psb = btSoftBodyHelpers::CreateFromTriMesh(pdemo->m_softBodyWorldInfo,  &gPoints[0],
                                                                 &gIndices[0],
                                                                 gNumofTri);
    btSoftBody::Material* pm = psb->appendMaterial();
    pm->m_kLST = 0.5;
    pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
    psb->generateBendingConstraints(2, pm);
    psb->m_cfg.piterations = 2;
    psb->m_cfg.kDF = 0.5;
    psb->randomizeConstraints();
    psb->scale(btVector3(1, 1, 1));
    psb->setTotalMass(100, true);
    pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

    pdemo->m_SoftBodies.push_back(psb);

    pdemo->m_cutting = true;

    gFuture = QtConcurrent::run([pdemo](){
        double freq = 1.0 / 60.0; //60Hz
        QElapsedTimer timer;
        timer.start();
        while (gRun) {
            pdemo->m_dynamicsWorld->stepSimulation(freq,1,freq);

            emit pdemo->glUpdateRequest();

            auto elapsedTime = timer.nsecsElapsed() * 1e-9;
            timer.restart();

            if ( freq > elapsedTime ) {
                auto runTime = freq - elapsedTime;
                QThread::usleep(runTime*1e6);
            }
        }
    });
}

static void Exit_Simulation(LP_Plugin_Simulator* pdemo){
    gRun = false;
    gFuture.waitForFinished();
    pdemo->exitPhysics();
}

void LP_Plugin_Simulator::initPhysics()
{
    ///create concave ground mesh
    btCollisionShape* groundShape = 0;
    {
        int i;
        int j;

        gGroundVertices = new btVector3[totalVerts];
        gGroundIndices = new int[totalTriangles * 3];

        btScalar offset(-50);

        for (i = 0; i < NUM_VERTS_X; i++)
        {
            for (j = 0; j < NUM_VERTS_Y; j++)
            {
                gGroundVertices[i + j * NUM_VERTS_X].setValue((i - NUM_VERTS_X * 0.5f) * TRIANGLE_SIZE,
                                                              //0.f,
                                                              waveheight * sinf((float)i) * cosf((float)j + offset),
                                                              (j - NUM_VERTS_Y * 0.5f) * TRIANGLE_SIZE);
//                qDebug() << gGroundVertices[i + j * NUM_VERTS_X].x()<<gGroundVertices[i + j * NUM_VERTS_X].y()<<gGroundVertices[i + j * NUM_VERTS_X].z();
            }
        }

        int vertStride = sizeof(btVector3);
        int indexStride = 3 * sizeof(int);

        int index = 0;
        for (i = 0; i < NUM_VERTS_X - 1; i++)
        {
            for (int j = 0; j < NUM_VERTS_Y - 1; j++)
            {
                gGroundIndices[index++] = j * NUM_VERTS_X + i;
                gGroundIndices[index++] = (j + 1) * NUM_VERTS_X + i + 1;
                gGroundIndices[index++] = j * NUM_VERTS_X + i + 1;
                ;

                gGroundIndices[index++] = j * NUM_VERTS_X + i;
                gGroundIndices[index++] = (j + 1) * NUM_VERTS_X + i;
                gGroundIndices[index++] = (j + 1) * NUM_VERTS_X + i + 1;
            }
        }

        btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles,
                                                                                       gGroundIndices,
                                                                                       indexStride,
                                                                                       totalVerts, (btScalar*)&gGroundVertices[0].x(), vertStride);

        bool useQuantizedAabbCompression = true;

        groundShape = new btBvhTriangleMeshShape(indexVertexArrays, useQuantizedAabbCompression);
        groundShape->setMargin(0.5);
    }

    m_collisionShapes.push_back(groundShape);

    btCollisionShape* groundBox = new btBoxShape(btVector3(100, CUBE_HALF_EXTENTS, 100));
    m_collisionShapes.push_back(groundBox);

    btCompoundShape* cylinderCompound = new btCompoundShape;
    btCollisionShape* cylinderShape = new btCylinderShape(btVector3(CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS));
    btTransform localTransform;
    localTransform.setIdentity();
    cylinderCompound->addChildShape(localTransform, cylinderShape);
    btQuaternion orn(btVector3(0, 1, 0), SIMD_PI);
    localTransform.setRotation(orn);
    cylinderCompound->addChildShape(localTransform, cylinderShape);

    m_collisionShapes.push_back(cylinderCompound);

    m_dispatcher = 0;

    ///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    m_softBodyWorldInfo.m_dispatcher = m_dispatcher;

    ////////////////////////////
    ///Register softbody versus softbody collision algorithm

    ///Register softbody versus rigidbody collision algorithm

    ////////////////////////////

//    btVector3 worldAabbMin(-1000, -1000, -1000);
//    btVector3 worldAabbMax(1000, 1000, 1000);

    //btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher);

    m_broadphase = new btDbvtBroadphase();

    m_softBodyWorldInfo.m_broadphase = m_broadphase;

    m_solver = new btMultiBodyConstraintSolver();

    btSoftBodySolver* softBodySolver = 0;
#ifdef USE_AMD_OPENCL

    static bool once = true;
    if (once)
    {
        once = false;
        initCL(0, 0);
    }

    if (g_openCLSIMDSolver)
        delete g_openCLSIMDSolver;
    if (g_softBodyOutput)
        delete g_softBodyOutput;

    if (1)
    {
        g_openCLSIMDSolver = new btOpenCLSoftBodySolverSIMDAware(g_cqCommandQue, g_cxMainContext);
        //	g_openCLSIMDSolver = new btOpenCLSoftBodySolver( g_cqCommandQue, g_cxMainContext);
        g_openCLSIMDSolver->setCLFunctions(new CachingCLFunctions(g_cqCommandQue, g_cxMainContext));
    }

    softBodySolver = g_openCLSIMDSolver;
    g_softBodyOutput = new btSoftBodySolverOutputCLtoCPU;
#endif  //USE_AMD_OPENCL

    btDiscreteDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration, softBodySolver);
    m_dynamicsWorld = world;

    m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
    m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
    m_softBodyWorldInfo.m_gravity.setValue(0, -10, 0);
    //	clientResetScene();

    m_softBodyWorldInfo.m_sparsesdf.Initialize();
    //	clientResetScene();

    //create ground object
    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(btVector3(0, 0, 0));

    btCollisionObject* newOb = new btCollisionObject();
    newOb->setWorldTransform(tr);
    newOb->setInterpolationWorldTransform(tr);

//    newOb->setCollisionShape(m_collisionShapes[0]);
    newOb->setCollisionShape(m_collisionShapes[1]);

    m_dynamicsWorld->addCollisionObject(newOb);

    m_softBodyWorldInfo.m_sparsesdf.Reset();

    m_softBodyWorldInfo.air_density = (btScalar)1.2;
    m_softBodyWorldInfo.water_density = 0;
    m_softBodyWorldInfo.water_offset = 0;
    m_softBodyWorldInfo.water_normal = btVector3(0, 0, 0);
    m_softBodyWorldInfo.m_gravity.setValue(0, -10, 0);

    m_cutting = false;
}

void LP_Plugin_Simulator::exitPhysics()
{
    //cleanup in the reverse order of creation/initialization

    //remove the rigidbodies from the dynamics world and delete them
    int i;
    for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        m_dynamicsWorld->removeCollisionObject(obj);
        delete obj;
    }

    //delete collision shapes
//    for (int j = 0; j < m_collisionShapes.size(); j++)
//    {
//        btCollisionShape* shape = m_collisionShapes[j];
//        m_collisionShapes[j] = 0;
//        delete shape;
//    }

    //delete dynamics world
    delete m_dynamicsWorld;
    m_dynamicsWorld = 0;

    //delete solver
    delete m_solver;

    //delete broadphase
    delete m_broadphase;

    //delete dispatcher
    delete m_dispatcher;

    delete m_collisionConfiguration;
}

LP_Plugin_Simulator::~LP_Plugin_Simulator()
{
    gRun = false;
    gFuture.waitForFinished();

    btAssert(m_dynamicsWorld == 0);

    // Clean the data
    emit glContextRequest([this](){
        delete mProgram_L;
        mProgram_L = nullptr;
    }, "Shade");

    emit glContextRequest([this](){
        delete mProgram_R;
        mProgram_R = nullptr;
    }, "Normal");

    Q_ASSERT(!mProgram_L);
    Q_ASSERT(!mProgram_R);
}

QWidget *LP_Plugin_Simulator::DockUi()
{
    mWidget = std::make_shared<QWidget>();
    QVBoxLayout *layout = new QVBoxLayout(mWidget.get());

    mObjectid = new QLabel("Object Index");
    mLabel = new QLabel("");
    mButton = new QPushButton("Start Simulation");

    layout->addWidget(mLabel);
    layout->addWidget(mObjectid);
    layout->addWidget(mButton);

    if(!gStarted){
        connect(mButton, &QPushButton::clicked, [this](){
            gStarted = true;
            Init_Cloth(this);
            mButton->setText("Exit Simulation");
        });
    } else if(gStarted) {
        connect(mButton, &QPushButton::clicked, [this](){
            gStarted = false;
            Exit_Simulation(this);
            gRun = true;
            mButton->setText("Start Simulation");
        });
    }


    mWidget->setLayout(layout);
    return mWidget.get();
}

bool LP_Plugin_Simulator::Run()
{
    gRun = true;
    gStarted = false;
    gSavedMesh = false;

    return false;
}

bool LP_Plugin_Simulator::eventFilter(QObject *watched, QEvent *event)
{
    if ( "window_Normal" == watched->objectName()){
        return QObject::eventFilter(watched, event);
    } // Else
    static auto _isMesh = [](LP_Objectw obj){
        if ( obj.expired()){
            return LP_OpenMeshw();
        }
        return LP_OpenMeshw() = std::static_pointer_cast<LP_OpenMeshImpl>(obj.lock());
    };


    if ( QEvent::MouseButtonRelease == event->type()){
        auto e = static_cast<QMouseEvent*>(event);

        if ( e->button() == Qt::LeftButton ){
            if (!mObject.lock()){
                auto Object = g_GLSelector->SelectInWorld("Shade",e->pos());
                if(Object.empty()){
                   return false;
                }
                auto c = _isMesh(Object.front());
                if ( c.lock()){
                    mObject = c;
                    mObjectid->setText(mObject.lock()->Uuid().toString());

                    emit glUpdateRequest();
                    return true;    //Since event filter has been called
                }
            } else if(!gSavedMesh){
                gSavedMesh = true;
                auto c = _isMesh(mObject).lock();

                auto sf_mesh = c->Mesh();
                auto sf_pt = sf_mesh->points();

                std::vector<double> Indices;
                for(auto i=0; i<sf_mesh->n_vertices(); i++){
                    auto pp = sf_pt[i];
                    gPoints.push_back(pp[0]);
                    gPoints.push_back(pp[1]+10);
                    gPoints.push_back(pp[2]);
//                    qDebug()<< gPoints[i*3] << gPoints[i*3+1] << gPoints[i*3+2];
                }
                Indices.resize(sf_mesh->n_faces()*3); // Triangular mesh
                auto i_it = Indices.begin();
                for ( const auto &f : sf_mesh->faces()){
                    for ( const auto &v : f.vertices()){
                        //Faces.emplace_back(v.idx());
                        (*i_it++) = v.idx();
                    }
                }
                for(auto i=0; i<Indices.size(); i++){
                        gIndices.push_back(Indices[i]);
//                        qDebug()<< Indices[i];
                }
                gNumofTri = sf_mesh->n_faces();
            }
        } else if ( e->button() == Qt::RightButton ){

        }
    return QObject::eventFilter(watched, event);
    }
}


void LP_Plugin_Simulator::FunctionalRender_L(QOpenGLContext *ctx, QSurface *surf, QOpenGLFramebufferObject *fbo, const LP_RendererCam &cam, const QVariant &options)
{
    Q_UNUSED(surf)  //Mostly not used within a Functional.
//    Q_UNUSED(options)   //Not used in this functional.

        if ( !mInitialized_L ){
            initializeGL_L();
        }

        QMatrix4x4 view = cam->ViewMatrix(),
                   proj = cam->ProjectionMatrix();


        auto f = ctx->extraFunctions();

        fbo->bind();
        mProgram_L->bind();
        mProgram_L->setUniformValue("m4_mvp", proj * view );
        mProgram_L->setUniformValue("m4_view", view);
        mProgram_L->setUniformValue("m3_normal", view.normalMatrix());
        mProgram_L->enableAttributeArray("a_pos");
        mProgram_L->enableAttributeArray("a_norm");
        //mProgram_L->enableAttributeArray("a_tex");

        std::vector<QVector3D> points, points_normal, ground, ground_normal;
        std::vector<uint> cloth_ids, ground_ids;

        //Draw the cloth
        mProgram_L->setUniformValue("vcolor", QVector3D(0.3, 0.7, 0.7));
        for ( auto &psb : m_SoftBodies ) {
            const btSoftBody::tNodeArray &particles = psb->m_nodes;

            const int nPts  = particles.size();
            for ( int i=0; i<nPts; ++i ) {
                const auto &pt = particles.at(i).m_x;
                const auto &pt2 = particles.at(i).m_n;
                QVector3D tmpPt(pt.x(), pt.y(), pt.z());
                QVector3D tmpPt2(pt2.x(), pt2.y(), pt2.z());
                points.push_back(tmpPt);
                points_normal.push_back(tmpPt2);
            }
            for ( int i=0; i<gIndices.size(); i++){
                cloth_ids.push_back(gIndices[i]);
            }
            mProgram_L->setAttributeArray("a_pos", points.data());
            mProgram_L->setAttributeArray("a_norm", points_normal.data());

            f->glDrawElements(GL_TRIANGLES, cloth_ids.size(), GL_UNSIGNED_INT, cloth_ids.data());
//            f->glDrawArrays(GL_POINTS, 0, nPts );
            points.clear();
            cloth_ids.clear();
            points_normal.clear();
        }

        if(gInitPhysics){
            //Draw the ground
            mProgram_L->setUniformValue("vcolor", QVector3D(0.0, 0.0, 0.0));
//            for (auto i=0; i<totalVerts; i++){
//                QVector3D tmpPt(gGroundVertices[i].x(), gGroundVertices[i].y(), gGroundVertices[i].z());
//                ground.push_back(tmpPt);
//            }
//            for (auto i=0; i<totalTriangles * 3; i++){
//                ground_ids.push_back(gGroundIndices[i]);
//            }

            for (int i=0; i<8 ; i++){
                btVector3 tmpPT;
                btBoxShape(btVector3(100, CUBE_HALF_EXTENTS, 100)).getVertex(i, tmpPT);
                ground.emplace_back(tmpPT.x(), tmpPT.y(), tmpPT.z());
//                qDebug()<< "i: "<< i<< "x: "<< tmpPT.x()<< "y: "<< tmpPT.y()<< "z: "<< tmpPT.z();
            }
            ground_ids.resize(36);
            ground_ids = {0,1,2,1,2,3,0,1,4,1,4,5,1,3,5,3,5,7,0,2,4,2,4,6,2,3,6,3,6,7,4,5,6,5,6,7};

            mProgram_L->setAttributeArray("a_pos", ground.data());
//            mProgram_L->setAttributeArray("a_norm", ground_normal.data());

            f->glDrawElements(GL_TRIANGLES, ground_ids.size(), GL_UNSIGNED_INT, ground_ids.data());
//            f->glDrawArrays(GL_POINTS, 0, ground.size() );
            ground.clear();
            ground_ids.clear();
//            ground_normal.clear();
        }

        mProgram_L->disableAttributeArray("a_norm");
        mProgram_L->disableAttributeArray("a_pos");
        //mProgram_L->disableAttributeArray("a_tex");
        mProgram_L->release();
        fbo->release();
}

void LP_Plugin_Simulator::FunctionalRender_R(QOpenGLContext *ctx, QSurface *surf, QOpenGLFramebufferObject *fbo, const LP_RendererCam &cam, const QVariant &options)
{
    Q_UNUSED(surf)  //Mostly not used within a Functional.
//    Q_UNUSED(options)   //Not used in this functional.

        if ( !mInitialized_R ){
            initializeGL_R();
        }

        QMatrix4x4 view = cam->ViewMatrix(),
                   proj = cam->ProjectionMatrix();

        auto f = ctx->extraFunctions();

        fbo->bind();
        mProgram_R->bind();

        mProgram_R->setUniformValue("m4_mvp", proj * view );
        mProgram_R->enableAttributeArray("a_pos");
        mProgram_R->enableAttributeArray("a_tex");

        mProgram_R->disableAttributeArray("a_pos");
        mProgram_R->disableAttributeArray("a_tex");
        mProgram_R->release();
        fbo->release();
}

void LP_Plugin_Simulator::initializeGL_L()
{
        std::string vsh, fsh;

            vsh =
                "attribute vec3 a_pos;\n"       //The position of a point in 3D that used in FunctionRender()
                "attribute vec3 a_norm;\n"
                "uniform mat4 m4_mvp;\n"        //The Model-View-Matrix
                "uniform mat4 m4_view;\n"
                "uniform mat3 m3_normal;\n"
                "varying vec3 pos;\n"
                "varying vec3 normal;\n"
                "void main(){\n"
                "   pos = vec3( m4_view * vec4(a_pos, 1.0));\n"
                "   normal = m3_normal * a_norm;\n"
                "   gl_Position = m4_mvp * vec4(a_pos, 1.0);\n" //Output the OpenGL position
                "   gl_PointSize = 10.0;\n"
                "}";
            fsh =
                "varying vec3 normal;\n"
                "varying vec3 pos;\n"
                "uniform vec3 vcolor;\n"
                "void main(){\n"
                "   vec3 lightPos = vec3(0.0, 1000.0, 0.0);\n"
                "   vec3 viewDir = normalize( - pos);\n"
                "   vec3 lightDir = normalize(lightPos - pos);\n"
                "   vec3 H = normalize(viewDir + lightDir);\n"
                "   vec3 N = normalize(normal);\n"
                "   vec3 ambi = vcolor;\n"
                "   float Kd = max(dot(H, N), 0.0);\n"
                "   vec3 diff = Kd * vec3(0.2, 0.2, 0.2);\n"
                "   vec3 color = ambi + diff;\n"
                "   float Ks = pow( Kd, 80.0 );\n"
                "   vec3 spec = Ks * vec3(0.5, 0.5, 0.5);\n"
                "   color += spec;\n"
                "   gl_FragColor = vec4(color, 1.0);\n" //Output the fragment color;
                "}";

        auto prog = new QOpenGLShaderProgram;   //Intialize the Shader with the above GLSL codes
        prog->addShaderFromSourceCode(QOpenGLShader::Vertex,vsh.c_str());
        prog->addShaderFromSourceCode(QOpenGLShader::Fragment,fsh.data());
        if (!prog->create() || !prog->link()){  //Check whether the GLSL codes are valid
            qDebug() << prog->log();
            return;
        }

        mProgram_L = prog;            //If everything is fine, assign to the member variable

        mInitialized_L = true;
}

void LP_Plugin_Simulator::initializeGL_R()
{
    std::string vsh, fsh;

        vsh =
            "attribute vec3 a_pos;\n"       //The position of a point in 3D that used in FunctionRender()
            "attribute vec2 a_tex;\n"
            "uniform mat4 m4_mvp;\n"        //The Model-View-Matrix
            "uniform float f_pointSize;\n"  //Point size determined in FunctionRender()
            "varying vec2 tex;\n"
            "void main(){\n"
            "   gl_Position = m4_mvp * vec4(a_pos, 1.0);\n" //Output the OpenGL position
            "   gl_PointSize = f_pointSize; \n"
            "   tex = a_tex;\n"
            "}";
        fsh =
            "uniform sampler2D u_tex;\n"    //Defined the point color variable that will be set in FunctionRender()
            "varying vec2 tex;\n"
            "void main(){\n"
            "   vec4 v4_color = texture2D(u_tex, tex);\n"
            "   gl_FragColor = v4_color;\n" //Output the fragment color;
            "}";

    auto prog = new QOpenGLShaderProgram;   //Intialize the Shader with the above GLSL codes
    prog->addShaderFromSourceCode(QOpenGLShader::Vertex,vsh.c_str());
    prog->addShaderFromSourceCode(QOpenGLShader::Fragment,fsh.data());
    if (!prog->create() || !prog->link()){  //Check whether the GLSL codes are valid
        qDebug() << prog->log();
        return;
    }

    mProgram_R = prog;            //If everything is fine, assign to the member variable

    mInitialized_R = true;
}

QString LP_Plugin_Simulator::MenuName()
{
    return tr("menuPlugins");
}

QAction *LP_Plugin_Simulator::Trigger()
{
    if ( !mAction ){
        mAction = new QAction("Simulator");
    }
    return mAction;
}
