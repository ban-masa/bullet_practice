#include <iostream>
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftBody.h>
#include <GL/glut.h>
#include "MyDebugDraw.h"

MyDebugDraw debug_draw;
btSoftRigidDynamicsWorld* dynamicsWorld;
btSoftBodyWorldInfo softBodyWorldInfo;
float pos[3] = {0};
int g_argc;
char** g_argv;
bool start_flag = false;

void keyboardCallback(unsigned char key, int x, int y)
{
  if (key == 27) exit(0);
  if (key == 'd') pos[0] = pos[0] + 0.5;
  if (key == 'a') pos[0] = pos[0] - 0.5;
  if (key == 'e') pos[1] = pos[1] + 0.5;
  if (key == 'q') pos[1] = pos[1] - 0.5;
  if (key == 'w') pos[2] = pos[2] + 0.5;
  if (key == 's') pos[2] = pos[2] - 0.5;
  if (key == ' ') start_flag = !start_flag;
}

void mouseCallback(int button, int state, int x, int y)
{
}

void idleCallback()
{
  //glutPostRedisplay();
}

void resize(int w, int h)
{
	glMatrixMode(GL_PROJECTION);
    glViewport(0, 0, w, h);
    glLoadIdentity();
    gluPerspective(30.0, (double)w / (double)h, 1.0, 5000.0);
    gluLookAt(0.0, 10.0, 100.0, 0.0, 10.0, -10.0, 0.0, 1.0, 0.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void timerFunc(int value)
{
  glutPostRedisplay();
  glutTimerFunc(33, timerFunc, value);
}

void renderCallback()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  glTranslatef(pos[0], pos[1], pos[2]);
  if (start_flag) dynamicsWorld->stepSimulation(1 / 60.f, 10);
  dynamicsWorld->debugDrawWorld();
  glutSwapBuffers();
  glFlush();
}


void createSoftSphere(void)
{
  btSoftBody* sphere = btSoftBodyHelpers::CreateEllipsoid(softBodyWorldInfo, btVector3(0, 0, 0), btVector3(2, 2, 2), 50);
  sphere->getCollisionShape()->setMargin(0.05);
  sphere->setTotalMass(1.0);
  sphere->m_materials[0]->m_kLST = 0.5;
  sphere->m_cfg.kVC = 1.0;
  sphere->setPose(true, false);
  sphere->translate(btVector3(0, 5, 0));
  dynamicsWorld->addSoftBody(sphere);
}

void createGround(void)
{
  btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
  btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
  btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
  btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
  groundRigidBody->setRestitution(1.0);
  dynamicsWorld->addRigidBody(groundRigidBody);
}

void createRigidSphere(void)
{
  btCollisionShape* fallShape = new btSphereShape(1);
  btDefaultMotionState* fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(-5, 20, 0)));
  btScalar mass = 1;
  btVector3 fallInertia(0, 0, 0);
  fallShape->calculateLocalInertia(mass, fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
  btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
  fallRigidBody->setRestitution(0.7);
  dynamicsWorld->addRigidBody(fallRigidBody);
}

void createRigidBox(btVector3 size, btVector3 pos, btScalar mass)
{
  btCollisionShape* boxshape = new btBoxShape(size);
  btDefaultMotionState* motionstate = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), pos));
  btVector3 inertia(0, 0, 0);
  boxshape->calculateLocalInertia(mass, inertia);
  btRigidBody::btRigidBodyConstructionInfo boxci(mass, motionstate, boxshape, inertia);
  btRigidBody* box = new btRigidBody(boxci);
  box->setRestitution(0.0);
  dynamicsWorld->addRigidBody(box);
}

void createCloth(void)
{
  btSoftBody* cloth = btSoftBodyHelpers::CreatePatch(softBodyWorldInfo, btVector3(-3, 5, -3), btVector3(3, 5, -3), btVector3(-3, 5, 3), btVector3(3, 5, 3), 31, 31, 1+2+4+8, true);
  cloth->getCollisionShape()->setMargin(0.01f);
  btSoftBody::Material* pm = cloth->appendMaterial();
  pm->m_kLST = 0.4;
  pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
  cloth->generateBendingConstraints(2, pm);
  cloth->setTotalMass(10);
  cloth->m_cfg.piterations = 5;
  cloth->m_cfg.kDP = 0.005f;
  cloth->translate(btVector3(-5, 0, 0));
  dynamicsWorld->addSoftBody(cloth);
}

void createRobotArm(void)
{
}

void initPhysics(void)
{
  btBroadphaseInterface* broadphase = new btDbvtBroadphase();
  btSoftBodyRigidBodyCollisionConfiguration* collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
  btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
  btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
  dynamicsWorld = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
  dynamicsWorld->setGravity(btVector3(0, -10, 0));

  createGround();

  createRigidBox(btVector3(5, 0.5, 5), btVector3(0, 1, 0), 10);
  createRigidBox(btVector3(3, 0.5, 3), btVector3(2, 10, 0), 0.3);

  createRigidSphere();

  //testcreate();

  softBodyWorldInfo.m_broadphase = broadphase;
  softBodyWorldInfo.m_dispatcher = dispatcher;
  softBodyWorldInfo.m_gravity = dynamicsWorld->getGravity();
  softBodyWorldInfo.m_sparsesdf.Initialize();

  createSoftSphere();

  createCloth();

  dynamicsWorld->setDebugDrawer(&debug_draw);
  dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawContactPoints);
}

void init(void)
{
//  static GLfloat position[] = {-10.0f, 10.0f, 10.0f, 1.0f};
//  static GLfloat ambient [] = { 0.0f, 0.0f, 0.0f, 1.0f};
//  static GLfloat diffuse [] = { 1.0f, 1.0f, 1.0f, 1.0f};
//  static GLfloat specular[] = { 1.0f, 1.0f, 1.0f, 1.0f};
//  glLightfv(GL_LIGHT0, GL_POSITION, position);
//  glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
//  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
//  glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
//  glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
//  glEnable(GL_LIGHTING);
//  glEnable(GL_LIGHT0);
}

void cleanup(void)
{
  for(int i = dynamicsWorld->getNumCollisionObjects()-1; i >= 0; --i){
    btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];

    btRigidBody* body = btRigidBody::upcast(obj);
    if(body && body->getMotionState()){
      delete body->getMotionState();
    }

    // オブジェクトがSoft Bodyの場合の破棄
    btSoftBody* softBody = btSoftBody::upcast(obj);
    if(softBody){
      static_cast<btSoftRigidDynamicsWorld*>(dynamicsWorld)->removeSoftBody(softBody);
    }
    else{
      static_cast<btSoftRigidDynamicsWorld*>(dynamicsWorld)->removeCollisionObject(obj);
    }

    dynamicsWorld->removeCollisionObject(obj);
    delete obj;
  }
}

int main(int argc, char** argv)
{
  g_argc = argc;
  g_argv = argv;
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH);
  glutCreateWindow(argv[0]);
  glutReshapeFunc(resize);
  glutDisplayFunc(renderCallback);
  glutIdleFunc(idleCallback);
  glutKeyboardFunc(keyboardCallback);
  glutMouseFunc(mouseCallback);
  init();

  initPhysics();
  glutTimerFunc(33, timerFunc, 0);
	atexit(cleanup);
  glutMainLoop();

//  delete fallRigidBody->getMotionState();
//  delete fallRigidBody;
//
//  dynamicsWorld->removeRigidBody(groundRigidBody);
//  delete groundRigidBody->getMotionState();
//  delete groundRigidBody;
//
//  delete fallShape;
//  delete groundShape;
//
//  delete dynamicsWorld;
//  delete solver;
//  delete dispatcher;
//  delete collisionConfiguration;
//  delete broadphase;

  return 0;
}
