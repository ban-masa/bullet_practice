#pragma once
#include <LinearMath/btIDebugDraw.h>
#include <GL/glut.h>
#include <iostream>

class MyDebugDraw : public btIDebugDraw {
public:
  void drawLine(const btVector3& from, const btVector3& to, const btVector3& color)
  {
    glColor3d(color.x(), color.y(), color.z());
    glBegin(GL_LINES);
    glVertex3d(from.x(), from.y(), from.z());
    glVertex3d(to.x(), to.y(), to.z());
    glEnd();
  }

  void drawLine(const btVector3& from, const btVector3& to, const btVector3& fromColor, const btVector3& toColor)
  {
    glBegin(GL_LINES);
    glColor3d(fromColor.x(), fromColor.y(), fromColor.z());
    glVertex3d(from.x(), from.y(), from.z());
    glColor3d(toColor.x(), toColor.y(), toColor.z());
    glVertex3d(to.x(), to.y(), to.z());
    glEnd();
  }

  void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color)
  {
    glPointSize(2);
    glColor3d(color.x(), color.y(), color.z());
    glBegin(GL_POINTS);
    glVertex3d(PointOnB.x(), PointOnB.y(), PointOnB.z());
    glEnd();
  }

  void reportErrorWarning(const char* warningString)
  {
    std::cerr << warningString << std::endl;
  }

  void draw3dText(const btVector3& location, const char* textString)
  {
    glRasterPos3f(location.x(), location.y(), location.z());
    for (int i = 0; textString[i] != '\0'; i++) {
      glutBitmapCharacter(GLUT_BITMAP_9_BY_15, textString[i]);
    }
  }

  int debug_mode;

  void setDebugMode(int debugMode)
  {
    debug_mode = debugMode;
  }

  int getDebugMode() const {
    return debug_mode;
  }
};

