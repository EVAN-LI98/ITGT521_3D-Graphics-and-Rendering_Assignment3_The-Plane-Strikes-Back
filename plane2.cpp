//|___________________________________________________________________
//!
//! \file plane2_base.cpp
//!
//! \brief Base source code for the second plane assignment.
//!
//! Author: Mores Prachyabrued.
//!
//! Keyboard inputs for plane and propeller (subpart):
//! ------------------------------------------------------> Plane 2 飞机2
//!   i   = moves the plane forward    i 移动飞机向前
//!   k   = moves the plane backward    k 移动飞机向后
//!   q,e = rolls the plane     q,e 旋转飞机
//!   a,d = yaws the plane     a,d 飞机偏航
//!   x,s = pitches the plane     x,s 飞机俯角
//!   r   = Rotates right propeller    r 旋转右边螺旋桨
//!   y   = Rotates left propeller    y 旋转左边螺旋桨
//!   t   = Rotates Front propeller    t 旋转前边螺旋桨
//!   g   = Rotates Subsubpropeller    g 旋转螺旋桨配件
//! ------------------------------------------------------> Plane 1 飞机1
//! //!   I   = moves the plane forward    I 移动飞机向前
//!   K   = moves the plane backward    K 移动飞机向后
//!   Q,E = rolls the plane     Q,E 旋转飞机
//!   A,D = yaws the plane     A,D 飞机偏航
//!   X,S = pitches the plane     X,S 飞机俯角
//! ------------------------------------------------------> Select Camera 选择相机
//!   v   = Select camera to view     v 选择相机视角
//!   b   = Select camera to control    b 选择相机控制
//!
//!   Mouse inputs for world-relative camera:
//!   Hold left button and drag  = controls azimuth and elevation   
//!   按住鼠标左键控制相机的方位角和仰角
//! 
//!   (Press CTRL (and hold) before left button to restrict to azimuth control only,
//!    Press SHIFT (and hold) before left button to restrict to elevation control only)  
//!   (按住 CTRL 并 拖动鼠标左键 可以实现相机方位角的固定旋转
//!    按住 SHIFT 并 拖动鼠标左键 可以实现相机仰角的固定旋转)
//!  
//!   Hold right button and drag = controls distance                
//!   按住鼠标右键控制相机的缩放距离
//! 
//! TODO: Extend the code to satisfy the requirements given in the assignment handout
//!
//! Note: Good programmer uses good comments! :)
//|___________________________________________________________________

//|___________________
//|
//| Includes
//|___________________

#include <math.h>

#include <gmtl/gmtl.h>

#include <GL/glut.h>

//|___________________
//|
//| Constants
//|___________________

// Plane dimensions
const float P_WIDTH        = 3;
const float P_LENGTH       = 3;
const float P_HEIGHT       = 1.5f;

// Plane transforms
const gmtl::Vec3f PLANE_FORWARD(0, 0, 1.0f);            // Plane's forward translation vector (w.r.t. local frame)
const float PLANE_ROTATION = 5.0f;                      // Plane rotated by 5 degs per input

// Propeller dimensions (subpart)
const float PP_WIDTH       = 0.25f;
const float PP_LENGTH      = 1.5f;

// Propeller transforms  螺旋桨的坐标位置
const gmtl::Point3f PROPELLER_POS(5, 0.25, 5.5);     // Propeller0 position on the plane (w.r.t. plane's frame) 
const gmtl::Point3f PROPELLER_POS1(-5,0.25,5.5);     // Propeller1 position on the plane (w.r.t. plane's frame) 
const gmtl::Point3f PROPELLER_POS2(0, 0, 0);         // Propeller2 position on the plane (w.r.t. plane's frame)
//const gmtl::Point3f PROPELLER_POS3(P_WIDTH / 2, 0, 0);         // Propeller2 position on the plane (w.r.t. plane's frame)
const float PROPELLER_ROTATION = 5.0f;               // Propeller rotated by 5 degs per input

// Camera's view frustum 
const float CAM_FOV        = 90.0f;                     // Field of view in degs

// Keyboard modifiers
enum KeyModifier {KM_SHIFT = 0, KM_CTRL, KM_ALT};

//|___________________
//|
//| Global Variables
//|___________________

// Track window dimensions, initialized to 800x600
int w_width    = 800;
int w_height   = 600;

// Plane pose (position-quaternion pair)
gmtl::Point4f plane_p;      // Position (using explicit homogeneous form; see Quaternion example code)
gmtl::Quatf plane_q;        // Quaternion

// Quaternions to rotate plane 1
gmtl::Quatf zrotp_q;        // Positive and negative Z rotations
gmtl::Quatf zrotn_q;

gmtl::Quatf xrotp_q;        // Positive and negative X rotations
gmtl::Quatf xrotn_q;

gmtl::Quatf yrotp_q;        // Positive and negative Y rotations
gmtl::Quatf yrotn_q;

// Plane pose 2 (position-quaternion pair)
gmtl::Point4f plane_p2;      // Position (using explicit homogeneous form; see Quaternion example code)
gmtl::Quatf plane_q2;        // Quaternion

// Quaternions to rotate plane 2
gmtl::Quatf zrotp_q2;        // Positive and negative Z rotations
gmtl::Quatf zrotn_q2;

gmtl::Quatf xrotp_q2;        // Positive and negative X rotations
gmtl::Quatf xrotn_q2; 

gmtl::Quatf yrotp_q2;        // Positive and negative Y rotations
gmtl::Quatf yrotn_q2;

// Propeller rotation (subpart)
float pp_angle = 0;         // Rotation right angle
float pp_angle_l = 0;         // Rotation front angle
float pp_angle_t = 0;         // Rotation left angle
float pp_angle_ss = 0;         // Rotation subpropeller angle

// Mouse & keyboard
int mx_prev = 0, my_prev = 0;
bool mbuttons[3]   = {false, false, false};
bool kmodifiers[3] = {false, false, false};

// Cameras
int cam_id         = 0;                                // Selects which camera to view
int camctrl_id     = 0;                                // Selects which camera to control
float distance[3]  = { 20.0f,  20.0f,  20.0f };                 // Distance of the camera from world's origin.
float elevation[3] = {-55.0f, -45.0f, -45.0f };                 // Elevation of the camera. (in degs)
float azimuth[3]   = { 10.0f,  15.0f,  20.0f };                 // Azimuth of the camera. (in degs)

/*
float distance[3]  = { 20.0f,  20.0f,  20.0f };                 // Distance of the camera from world's origin.
float elevation[3] = {-45.0f, -45.0f, -45.0f };                 // Elevation of the camera. (in degs)
float azimuth[3]   = { 15.0f,  15.0f,  15.0f };                 // Azimuth of the camera. (in degs)
*/

//|___________________
//|
//| Function Prototypes
//|___________________

void InitTransforms();
void InitGL(void);
void DisplayFunc(void);
void KeyboardFunc(unsigned char key, int x, int y);
void MouseFunc(int button, int state, int x, int y);
void MotionFunc(int x, int y);
void ReshapeFunc(int w, int h);
void DrawCoordinateFrame(const float l);
void DrawPlaneBody(const float width, const float length, const float height);
void DrawPlaneBody2(const float width, const float length, const float height);
void DrawPropeller(const float width, const float length);
void DrawPropeller2(const float width, const float length);
void SubSubDrawPropeller(const float width, const float length);

//|____________________________________________________________________
//|
//| Function: InitTransforms
//|
//! \param None.
//! \return None.
//!
//! Initializes all the transforms
//|____________________________________________________________________

void InitTransforms()
{
  const float COSTHETA_D2  = cos(gmtl::Math::deg2Rad(PLANE_ROTATION/2));  // cos() and sin() expect radians 
  const float SINTHETA_D2  = sin(gmtl::Math::deg2Rad(PLANE_ROTATION/2));

  // Inits plane2 pose
  plane_p.set(0.0f, 0.0f, 0.0f, 1.0f);
  plane_q.set(0, 0, 0, 1);

  // Z rotations (roll)
  zrotp_q.set(0, 0, SINTHETA_D2, COSTHETA_D2);      // +Z
  zrotn_q = gmtl::makeConj(zrotp_q);                // -Z

  // X rotation (pitch)
  xrotp_q.set(SINTHETA_D2, 0, 0, COSTHETA_D2);      // +X
  xrotn_q = gmtl::makeConj(xrotp_q);                // -X

  // Y rotation (yaw)
  yrotp_q.set(0, SINTHETA_D2, 0, COSTHETA_D2);      // +Y
  yrotn_q = gmtl::makeConj(yrotp_q);                // -Y
  
  // TODO: Initializes the remaining transforms

  // Inits plane1 pose
  plane_p2.set(-5.0f, 0.0f, 25.0f, 1.0f);
  plane_q2.set(0, 0, 0, 1);

  // Z rotations (roll)
  zrotp_q2.set(0, 0, SINTHETA_D2, COSTHETA_D2);      // +Z
  zrotn_q2 = gmtl::makeConj(zrotp_q2);                // -Z

  // X rotation (pitch)
  xrotp_q2.set(SINTHETA_D2, 0, 0, COSTHETA_D2);      // +X
  xrotn_q2 = gmtl::makeConj(xrotp_q2);                // -X

  // Y rotation (yaw)
  yrotp_q2.set(0, SINTHETA_D2, 0, COSTHETA_D2);      // +Y
  yrotn_q2 = gmtl::makeConj(yrotp_q2);                // -Y
}

//|____________________________________________________________________
//|
//| Function: InitGL
//|
//! \param None.
//! \return None.
//!
//! OpenGL initializations
//|____________________________________________________________________

void InitGL(void)
{
  glClearColor(0.7f, 0.7f, 0.7f, 1.0f); 
  glEnable(GL_DEPTH_TEST); 
  glShadeModel(GL_SMOOTH);
}

//|____________________________________________________________________
//|
//| Function: DisplayFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT display callback function: called for every redraw event.
//|____________________________________________________________________

void DisplayFunc(void)
{
  gmtl::AxisAnglef aa;    // Converts plane2's quaternion to axis-angle form to be used by glRotatef()
  gmtl::AxisAnglef aa2;    // Converts plane1's quaternion to axis-angle form to be used by glRotatef()
  gmtl::Vec3f axis;       // Plane2 Axis component of axis-angle representation
  gmtl::Vec3f axis2;       // Plane1 Axis component of axis-angle representation
  float angle;            // Plane2 Angle component of axis-angle representation
  float angle2;            // Plane1 Angle2 component of axis-angle representation

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(CAM_FOV, (float)w_width/w_height, 0.1f, 1000.0f);     // Check MSDN: google "gluPerspective msdn"

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

//|____________________________________________________________________
//|
//| Setting up view transform by:
//| "move up to the world frame by composing all of the (inverse) transforms from the camera up to the world node"
//|____________________________________________________________________

  switch (cam_id) {
    case 0:
      // For the world-relative camera 坐标轴的摄像机
      glTranslatef(0, 0, -distance[0]);
      glRotatef(-elevation[0], 1, 0, 0);
      glRotatef(-azimuth[0], 0, 1, 0);
      break;

    case 1:
      // For plane2's camera 飞机2的摄像机
      glTranslatef(0, 0, -distance[1]);
      glRotatef(-elevation[1], 1, 0, 0);
      glRotatef(-azimuth[1], 0, 1, 0);

      gmtl::set(aa, plane_q);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
      axis  = aa.getAxis();
      angle = aa.getAngle();
      glRotatef(-gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
      glTranslatef(-plane_p[0], -plane_p[1], -plane_p[2]);      
      break;

      // TODO: Add case for the plane1's camera

    case 2:
      // For plane1's camera 飞机1的摄像机
      glTranslatef(0, 0, -distance[2]);
      glRotatef(-elevation[2], 1, 0, 0);
      glRotatef(-azimuth[2], 0, 1, 0);

      gmtl::set(aa2, plane_q2);                 // Converts plane's quaternion to axis-angle form to be used by glRotatef()
      axis2 = aa2.getAxis();
      angle2 = aa2.getAngle();
      glRotatef(-gmtl::Math::rad2Deg(angle2), axis2[0], axis2[1], axis2[2]);
      glTranslatef(-plane_p2[0], -plane_p2[1], -plane_p2[2]);
      break;
  }

//|____________________________________________________________________
//|
//| Draw traversal begins, start from world (root) node
//|____________________________________________________________________

  // World node: draws world coordinate frame
  DrawCoordinateFrame(10);

  // World-relative camera:
  //摄像机1
  if (cam_id != 0) {
    glPushMatrix();        
      glRotatef(azimuth[0], 0, 1, 0);
      glRotatef(elevation[0], 1, 0, 0);
      glTranslatef(0, 0, distance[0]);
      DrawCoordinateFrame(1);
    glPopMatrix();
  }

  // Plane 2 body:
  glPushMatrix();
    gmtl::set(aa, plane_q);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
    axis  = aa.getAxis();
    angle = aa.getAngle();
    glTranslatef(plane_p[0], plane_p[1], plane_p[2]);
    glRotatef(gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
    DrawPlaneBody(P_WIDTH, P_LENGTH, P_HEIGHT);
    DrawCoordinateFrame(3);

    // Plane 2's camera:
    //摄像机2
    glPushMatrix();
    glRotatef(azimuth[1], 0, 1, 0);
    glRotatef(elevation[1], 1, 0, 0);
    glTranslatef(0, 0, distance[1]);
    DrawCoordinateFrame(3);
    glPopMatrix();

    //right Propeller (subpart):
    glPushMatrix();
      glTranslatef(PROPELLER_POS1[0], PROPELLER_POS1[1], PROPELLER_POS1[2]);     // Positions right propeller on the plane
      glRotatef(pp_angle, 0, 1, 0);                                           // Rotates propeller
      DrawPropeller(PP_WIDTH, PP_LENGTH);
      DrawCoordinateFrame(2);
    glPopMatrix();
 
    //left Propeller (subpart):
    glPushMatrix();
      glTranslatef(PROPELLER_POS[0], PROPELLER_POS[1], PROPELLER_POS[2]);     // Positions left propeller on the plane
      glRotatef(-pp_angle_t, 0, 1, 0);                                           // Rotates propeller
      DrawPropeller(PP_WIDTH, PP_LENGTH);
      DrawCoordinateFrame(2);
    glPopMatrix();

    //Front Propeller (subpart):
    glPushMatrix();
      glTranslatef(PROPELLER_POS2[0], PROPELLER_POS2[1], PROPELLER_POS2[2]);     // Positions front propeller on the plane
      glRotatef(pp_angle_l, 0, 0, 1);                                           // Rotates propeller
      DrawPropeller2(PP_WIDTH, PP_LENGTH);
      DrawCoordinateFrame(2);

      // Subsubpart:
      glPushMatrix();
      glTranslatef(PROPELLER_POS2[0], PROPELLER_POS2[1], PROPELLER_POS2[2]);                                        // Positions subpropeller on the plane
      glRotatef(pp_angle_ss, 0, 0, 1);
      SubSubDrawPropeller(PP_WIDTH, PP_LENGTH);
      DrawCoordinateFrame(1);
      glPopMatrix();
    glPopMatrix();
  glPopMatrix();

  // Plane 1 body:
  glPushMatrix();
    gmtl::set(aa, plane_q2);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
    axis = aa.getAxis();
    angle = aa.getAngle();
    glTranslatef(plane_p2[0], plane_p2[1], plane_p2[2]);
    glRotatef(gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
    DrawPlaneBody(P_WIDTH, P_LENGTH, P_HEIGHT);
    DrawCoordinateFrame(3);

    // Plane 1's camera:
    //摄像机3
    glPushMatrix();
    glRotatef(azimuth[2], 0, 1, 0);
    glRotatef(elevation[2], 1, 0, 0);
    glTranslatef(0, 0, distance[2]);
    DrawCoordinateFrame(1);
    glPopMatrix();
  glPopMatrix();

  glutSwapBuffers();                          // Replaces glFlush() to use double buffering
}

//|____________________________________________________________________
//|
//| Function: KeyboardFunc
//|
//! \param key    [in] Key code.
//! \param x      [in] X-coordinate of mouse when key is pressed.
//! \param y      [in] Y-coordinate of mouse when key is pressed.
//! \return None.
//!
//! GLUT keyboard callback function: called for every key press event.
//|____________________________________________________________________

void KeyboardFunc(unsigned char key, int x, int y)
{
  switch (key) {
//|____________________________________________________________________
//|
//| Camera switch
//|____________________________________________________________________

    case 'v': // Select camera to view
      cam_id = (cam_id + 1) % 3;
      printf("View camera = %d\n", cam_id);
      break;
    case 'b': // Select camera to control
      camctrl_id = (camctrl_id + 1) % 3;
      printf("Control camera = %d\n", camctrl_id);
      break;

//|____________________________________________________________________
//|
//| Plane controls
//|____________________________________________________________________
    
    //飞机1控制部分
    case 'I': { // Forward translation of the plane (+Z translation)  
        gmtl::Quatf v_q = plane_q2 * gmtl::Quatf(PLANE_FORWARD[0], PLANE_FORWARD[1], PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q);
        plane_p2 = plane_p2 + v_q.mData;
    } break;
    case 'K': { // Backward translation of the plane (-Z translation)
        gmtl::Quatf v_q = plane_q2 * gmtl::Quatf(-PLANE_FORWARD[0], -PLANE_FORWARD[1], -PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q);
        plane_p2 = plane_p2 + v_q.mData;
    } break;

    case 'E': // Rolls the plane (+Z rot) //逆时针旋转飞机
        plane_q2 = plane_q2 * zrotp_q;
        break;
    case 'Q': // Rolls the plane (-Z rot) //顺时针旋转飞机
        plane_q2 = plane_q2 * zrotn_q;
        break;

    case 'X': // Pitches the plane (+X rot) //飞机仰角
        plane_q2 = plane_q2 * xrotp_q;
        break;
    case 'S': // Pitches the plane (-X rot) //飞机俯角
        plane_q2 = plane_q2 * xrotn_q;
        break;

    case 'D': // Yaws the plane (+Y rot)  // 左转飞机
        plane_q2 = plane_q2 * yrotp_q;
        break;
    case 'A': // Yaws the plane (-Y rot)  // 右转飞机
        plane_q2 = plane_q2 * yrotn_q;
        break;
    

   //飞机2控制部分
    case 'i': { // Forward translation of the plane (+Z translation)  
      gmtl::Quatf v_q = plane_q * gmtl::Quatf(PLANE_FORWARD[0], PLANE_FORWARD[1], PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q);
      plane_p         = plane_p + v_q.mData;
      } break;
    case 'k': { // Backward translation of the plane (-Z translation)
      gmtl::Quatf v_q = plane_q * gmtl::Quatf(-PLANE_FORWARD[0], -PLANE_FORWARD[1], -PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q);
      plane_p         = plane_p + v_q.mData;
      } break;

    case 'e': // Rolls the plane (+Z rot) //逆时针旋转飞机
      plane_q = plane_q * zrotp_q;
      break;
    case 'q': // Rolls the plane (-Z rot) //顺时针旋转飞机
      plane_q = plane_q * zrotn_q;
      break;

    case 'x': // Pitches the plane (+X rot) //飞机仰角
      plane_q = plane_q * xrotp_q;
      break;
    case 's': // Pitches the plane (-X rot) //飞机俯角
      plane_q = plane_q * xrotn_q;
      break;

    case 'd': // Yaws the plane (+Y rot)  // 左转飞机
      plane_q = plane_q * yrotp_q;
      break;
    case 'a': // Yaws the plane (-Y rot)  // 右转飞机
      plane_q = plane_q * yrotn_q;
      break;

//|____________________________________________________________________
//|
//| Propeller controls (subpart)
//|____________________________________________________________________

    case 'r': // Rotates right propeller 
      pp_angle += PROPELLER_ROTATION;
      break;

    case 't': // Rotates front propeller 
        pp_angle_l += PROPELLER_ROTATION;
        break;

    case 'y': // Rotates left propeller 
        pp_angle_t += PROPELLER_ROTATION;
        break;

    case 'g': // Rotates subsubpart (propeller)
        pp_angle_ss += -PROPELLER_ROTATION;
        break;

    // TODO: Add the remaining controls/transforms        
  }

  glutPostRedisplay();                    // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: MouseFunc
//|
//! \param button     [in] one of GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON, or GLUT_RIGHT_BUTTON.
//! \param state      [in] one of GLUT_UP (event is due to release) or GLUT_DOWN (press).
//! \param x          [in] X-coordinate of mouse when an event occured.
//! \param y          [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT mouse-callback function: called for each mouse click.
//|____________________________________________________________________

void MouseFunc(int button, int state, int x, int y)
{
  int km_state;

  // Updates button's sate and mouse coordinates
  if (state == GLUT_DOWN) {
    mbuttons[button] = true;
    mx_prev          = x;
    my_prev          = y;
  } else {
    mbuttons[button] = false;
  }

  // Updates keyboard modifiers
  km_state = glutGetModifiers();
  kmodifiers[KM_SHIFT] = km_state & GLUT_ACTIVE_SHIFT ? true : false;
  kmodifiers[KM_CTRL]  = km_state & GLUT_ACTIVE_CTRL  ? true : false;
  kmodifiers[KM_ALT]   = km_state & GLUT_ACTIVE_ALT   ? true : false;

  //glutPostRedisplay();      // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: MotionFunc
//|
//! \param x      [in] X-coordinate of mouse when an event occured.
//! \param y      [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT motion-callback function: called for each mouse motion.
//|____________________________________________________________________

void MotionFunc(int x, int y)
{
  int dx, dy, d;

  if (mbuttons[GLUT_LEFT_BUTTON] || mbuttons[GLUT_RIGHT_BUTTON]) {
    // Computes distances the mouse has moved
    dx      = x - mx_prev;
    dy      = y - my_prev;

    // Updates mouse coordinates
    mx_prev = x;
    my_prev = y;

    // Hold left button to rotate camera
    if (mbuttons[GLUT_LEFT_BUTTON]) {
      if (!kmodifiers[KM_CTRL]) {        
        elevation[camctrl_id] += dy;            // Elevation update
      }
      if (!kmodifiers[KM_SHIFT]) {      
        azimuth[camctrl_id] += dx;             // Azimuth update
      }
    }

    // Hold right button to zoom
    if (mbuttons[GLUT_RIGHT_BUTTON]) {
      if (abs(dx) >= abs(dy)) {
        d = dx;
      } else {
        d = -dy;
      }
      distance[camctrl_id] += d;    
    }

    glutPostRedisplay();      // Asks GLUT to redraw the screen
  }
}

//|____________________________________________________________________
//|
//| Function: ReshapeFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT reshape callback function: called everytime the window is resized.
//|____________________________________________________________________

void ReshapeFunc(int w, int h)
{
  // Track the current window dimensions
  w_width  = w;
  w_height = h;
  glViewport(0, 0, (GLsizei) w_width, (GLsizei) w_height);
}

//|____________________________________________________________________
//|
//| Function: DrawCoordinateFrame
//|
//! \param l      [in] length of the three axes.
//! \return None.
//!
//! Draws coordinate frame consisting of the three principal axes.
//|____________________________________________________________________

void DrawCoordinateFrame(const float l)
{
    //坐标轴，X轴为红色，Y轴为绿色，Z轴为蓝色，(0,0)为坐标原点

  glBegin(GL_LINES);
    // X axis is red
    glColor3f( 1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
	  glVertex3f(   l, 0.0f, 0.0f);

    // Y axis is green
    glColor3f( 0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
	  glVertex3f(0.0f,    l, 0.0f);

    // Z axis is blue
    glColor3f( 0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
	  glVertex3f(0.0f, 0.0f,    l);
  glEnd();
}

//|____________________________________________________________________
//|
//| Function: DrawPlaneBody
//|
//! \param width       [in] Width  of the plane.
//! \param length      [in] Length of the plane.
//! \param height      [in] Height of the plane.
//! \return None.
//!
//! Draws a plane body.
//|____________________________________________________________________

void DrawPlaneBody(const float width, const float length, const float height)
{
    float w = width / 2;
    float l = length / 2;

    /*绘制飞机
    1,飞机尾部,上下两个部分 蓝白
    2,飞机尾翼,左右两个部分 白
    3,飞机机头,红黑
    4,飞机机舱,蓝色
    5,飞机机身,红黑
    6,飞机机翼,红色
    7,飞机后翼.黄色
    */
    glBegin(GL_POLYGON);
    //1
    //竖着的下部分
    glColor3f(1.0f, 1.0f, 1.0f);//颜色是白色white
    glVertex3f(0.0f, 1.0f, 0.5f);//
    glVertex3f(0.0f, 0.25f, 1.0f);//
    glVertex3f(0.0f, 0.5f, 4.0f);//
    glVertex3f(0.0f, 1.4f, 2.0f);//
    glColor3f(1.0f, 1.0f, 1.0f);//颜色是白色white

    //竖着的上部分
    glColor3f(0.0f, 3.0f, 1.0f);//颜色是蓝色blue
    glVertex3f(0.0f, 1.4f, 2.0f);//
    glVertex3f(0.0f, 3.0f, 1.0f);//
    glVertex3f(0.0f, 3.0f, 0.0f);//
    glVertex3f(0.0f, 1.0f, 0.5f);//
    glColor3f(0.0f, 3.0f, 1.0f);//颜色是蓝色blue
    glEnd();

    //2
    glBegin(GL_POLYGON);
    //横着的右半部分
    glColor3f(1.0f, 1.0f, 1.0f);//颜色是白white
    glVertex3f(0.0f, 2.0f, 0.25f);//
    glVertex3f(1.0f, 2.0f, -0.5f);//
    glVertex3f(2.0f, 2.0f, -0.5f);//
    glVertex3f(0.0f, 2.0f, 1.40f);//

    //横着的左半部分
    glColor3f(1.0f, 1.0f, 1.0f);//颜色是白white
    glVertex3f(0.0f, 2.0f, 0.25f);//
    glVertex3f(-1.0f, 2.0f, -0.5f);//
    glVertex3f(-2.0f, 2.0f, -0.5f);//
    glVertex3f(0.0f, 2.0f, 1.40f);//
    glEnd();

    //3
    //机头部分
    glBegin(GL_TRIANGLES);
    glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
    glVertex3f(-1.0f, 0.5f, 10.0f);//
    glVertex3f(1.0f, 0.5f, 10.0f);//
    glVertex3f(0.0f, 0.0f, 13.5f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
    glVertex3f(1.0f, 0.5f, 10.0f);//
    glVertex3f(2.0f, -0.2f, 10.0f);//
    glVertex3f(0.0f, 0.0f, 13.5f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
    glVertex3f(2.0f, -0.2f, 10.0f);//
    glVertex3f(-2.0f, -0.2f, 10.0f);//
    glVertex3f(0.0f, 0.0f, 13.5f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
    glVertex3f(-2.0f, -0.2f, 10.0f);//
    glVertex3f(-1.0f, 0.5f, 10.0f);//
    glVertex3f(0.0f, 0.0f, 13.5f);//
    glEnd();

    //4
    //飞机驾驶舱
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(0.0f, 0.5f, 10.0f);//
    glVertex3f(0.25f, 1.5f, 9.25f);//
    glVertex3f(-0.25f, 1.5f, 9.25f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(0.0f, 0.5f, 10.0f);//
    glVertex3f(0.25f, 1.5f, 9.25f);//
    glVertex3f(1.0f, 0.5f, 9.0f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(0.0f, 0.5f, 10.0f);//
    glVertex3f(-0.25f, 1.5f, 9.25f);//
    glVertex3f(-1.0f, 0.5f, 9.0f);//
    glEnd();

    glBegin(GL_QUADS);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(-1.0f, 0.5f, 9.0f);//
    glVertex3f(-0.25f, 1.5f, 9.25f);//
    glVertex3f(-0.25f, 1.5f, 7.75f);//
    glVertex3f(-1.0f, 0.5f, 8.0f);//
    glEnd();
    glBegin(GL_QUADS);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(0.25f, 1.5f, 9.25f);//
    glVertex3f(-0.25f, 1.5f, 9.25f);//
    glVertex3f(-0.25f, 1.5f, 7.75f);//
    glVertex3f(0.25f, 1.5f, 7.75f);//
    glEnd();
    glBegin(GL_QUADS);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(1.0f, 0.5f, 9.0f);//
    glVertex3f(0.25f, 1.5f, 9.25f);//
    glVertex3f(0.25f, 1.5f, 7.75f);//
    glVertex3f(1.0f, 0.5f, 8.0f);//
    glEnd();

    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(0.0f, 0.5f, 7.0f);//
    glVertex3f(0.25f, 1.5f, 7.75f);//
    glVertex3f(-0.25f, 1.5f, 7.75f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(0.0f, 0.5f, 7.0f);//
    glVertex3f(0.25f, 1.5f, 7.75f);//
    glVertex3f(1.0f, 0.5f, 8.0f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(0.0f, 0.5f, 7.0f);//
    glVertex3f(-0.25f, 1.5f, 7.75f);//
    glVertex3f(-1.0f, 0.5f, 8.0f);//
    glEnd();

    //5
    //机身
    glBegin(GL_QUADS);
    glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
    glVertex3f(-0.25f, 0.25f, 1.0f);//
    glVertex3f(0.25f, 0.25f, 1.0f);//
    glVertex3f(0.6f, 0.5f, 4.0f);//
    glVertex3f(-0.6f, 0.5f, 4.0f);//
    glEnd();
    glBegin(GL_QUADS);
    glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
    glVertex3f(0.6f, 0.5f, 4.0f);//
    glVertex3f(-0.6f, 0.5f, 4.0f);//
    glVertex3f(-1.0f, 0.5f, 10.0f);//
    glVertex3f(1.0f, 0.5f, 10.0f);//
    glEnd();

    glBegin(GL_QUADS);
    glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
    glVertex3f(-0.5f, -0.2f, -1.0f);//
    glVertex3f(-0.25f, 0.2f, 1.0f);//
    glVertex3f(0.25f, 0.2f, 1.0f);//
    glVertex3f(0.5f, -0.2f, -1.0f);//
    glEnd();
    glBegin(GL_POLYGON);
    glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
    glVertex3f(2.0f, -0.2f, 10.0f);//
    glVertex3f(1.0f, 0.5f, 10.0f);//
    glVertex3f(0.6f, 0.5f, 4.0f);//
    glVertex3f(0.25f, 0.2f, 1.0f);//
    glVertex3f(0.5f, -0.2f, -1.0f);//
    glVertex3f(0.5f, -0.2f, -1.0f);//
    glVertex3f(1.2f, -0.2f, 4.0f);//
    glEnd();
    glBegin(GL_POLYGON);
    glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
    glVertex3f(0.0f, 0.0f, 13.5f);//
    glVertex3f(-2.0f, -0.2f, 10.0f);//
    glVertex3f(-0.5f, -0.2f, -1.0f);//
    glVertex3f(0.5f, -0.2f, -1.0f);//
    glVertex3f(2.0f, -0.2f, 10.0f);//
    glEnd();

    glBegin(GL_POLYGON);
    glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
    glVertex3f(-2.0f, -0.2f, 10.0f);//
    glVertex3f(-1.0f, 0.5f, 10.0f);//
    glVertex3f(-0.6f, 0.5f, 4.0f);//
    glVertex3f(-0.25f, 0.2f, 1.0f);//
    glVertex3f(-0.5f, -0.2f, -1.0f);//
    glVertex3f(-0.5f, -0.2f, -1.0f);//
    glVertex3f(-1.2f, -0.2f, 4.0f);//
    glEnd();

    //6
    //飞机机翼
    glBegin(GL_POLYGON);
    glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red 
    glVertex3f(-1.75f, 0.0f, 10.0f);//
    glVertex3f(-5.0f, 0.0f, 7.5f);//
    glVertex3f(-5.0f, 0.25f, 4.1f);//
    glVertex3f(-1.0f, 0.25f, 4.0f);//
    glVertex3f(-1.75f, 0.0f, 10.0f);//
    glEnd();
    glBegin(GL_POLYGON);
    glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
    glVertex3f(-5.0f, 0.0f, 7.5f);//
    glVertex3f(-5.0f, 0.25f, 4.1f);//
    glVertex3f(-9.0f, 0.45f, 2.5f);//
    glVertex3f(-10.0f, 0.2f, 5.5f);//
    glEnd();
    glBegin(GL_POLYGON);

    glBegin(GL_POLYGON);
    glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red 
    glVertex3f(1.75f, 0.0f, 10.0f);//
    glVertex3f(5.0f, 0.0f, 7.5f);//
    glVertex3f(5.0f, 0.25f, 4.1f);//
    glVertex3f(1.0f, 0.25f, 4.0f);//
    glVertex3f(1.75f, 0.0f, 10.0f);//
    glEnd();
    glBegin(GL_POLYGON);
    glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
    glVertex3f(5.0f, 0.0f, 7.5f);//
    glVertex3f(5.0f, 0.25f, 4.1f);//
    glVertex3f(9.0f, 0.45f, 2.5f);//
    glVertex3f(10.0f, 0.2f, 5.5f);//
    glEnd();

    //7
    //飞机后翼
    glBegin(GL_POLYGON);
    glColor3f(1.0f, 1.0f, 0.0f);//颜色是黄色yellow
    glVertex3f(-0.5f, -0.25f, -1.0f);//
    glVertex3f(-0.55f, 0.1f, 1.0f);//
    glVertex3f(-2.75f, 0.1f, -0.25f);//
    glVertex3f(-2.5f, -0.25f, -1.1f);//
    glEnd();
    glBegin(GL_POLYGON);
    glColor3f(1.0f, 1.0f, 0.0f);//颜色是黄色yellow
    glVertex3f(0.5f, -0.25f, -1.0f);//
    glVertex3f(0.55f, 0.1f, 1.0f);//
    glVertex3f(2.75f, 0.1f, -0.25f);//
    glVertex3f(2.5f, -0.25f, -1.1f);//
    glEnd();

    /*
    glBegin(GL_POLYGON); 多边形
    glBegin(GL_QUADS); 四边形
    glBegin(GL_TRIANGLES); 三角形
    glEnd(); 结束
    */
}

void DrawPlaneBody2(const float width, const float length, const float height)
{
      float w = width / 2;
      float l = length / 2;

      /*绘制飞机
      1,飞机尾部,上下两个部分 蓝白
      2,飞机尾翼,左右两个部分 白
      3,飞机机头,红黑
      4,飞机机舱,蓝色
      5,飞机机身,红黑
      6,飞机机翼,红色
      7,飞机后翼.黄色
      */
      glBegin(GL_POLYGON);
      //1
      //竖着的下部分
      glColor3f(1.0f, 1.0f, 1.0f);//颜色是白色white
      glVertex3f(0.0f, 1.0f, 0.5f);//
      glVertex3f(0.0f, 0.25f, 1.0f);//
      glVertex3f(0.0f, 0.5f, 4.0f);//
      glVertex3f(0.0f, 1.4f, 2.0f);//
      glColor3f(1.0f, 1.0f, 1.0f);//颜色是白色white

      //竖着的上部分
      glColor3f(0.0f, 3.0f, 1.0f);//颜色是蓝色blue
      glVertex3f(0.0f, 1.4f, 2.0f);//
      glVertex3f(0.0f, 3.0f, 1.0f);//
      glVertex3f(0.0f, 3.0f, 0.0f);//
      glVertex3f(0.0f, 1.0f, 0.5f);//
      glColor3f(0.0f, 3.0f, 1.0f);//颜色是蓝色blue
      glEnd();

      //2
      glBegin(GL_POLYGON);
      //横着的右半部分
      glColor3f(1.0f, 1.0f, 1.0f);//颜色是白white
      glVertex3f(0.0f, 2.0f, 0.25f);//
      glVertex3f(1.0f, 2.0f, -0.5f);//
      glVertex3f(2.0f, 2.0f, -0.5f);//
      glVertex3f(0.0f, 2.0f, 1.40f);//

      //横着的左半部分
      glColor3f(1.0f, 1.0f, 1.0f);//颜色是白white
      glVertex3f(0.0f, 2.0f, 0.25f);//
      glVertex3f(-1.0f, 2.0f, -0.5f);//
      glVertex3f(-2.0f, 2.0f, -0.5f);//
      glVertex3f(0.0f, 2.0f, 1.40f);//
      glEnd();

      //3
      //机头部分
      glBegin(GL_TRIANGLES);
      glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
      glVertex3f(-1.0f, 0.5f, 10.0f);//
      glVertex3f(1.0f, 0.5f, 10.0f);//
      glVertex3f(0.0f, 0.0f, 13.5f);//
      glEnd();
      glBegin(GL_TRIANGLES);
      glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
      glVertex3f(1.0f, 0.5f, 10.0f);//
      glVertex3f(2.0f, -0.2f, 10.0f);//
      glVertex3f(0.0f, 0.0f, 13.5f);//
      glEnd();
      glBegin(GL_TRIANGLES);
      glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
      glVertex3f(2.0f, -0.2f, 10.0f);//
      glVertex3f(-2.0f, -0.2f, 10.0f);//
      glVertex3f(0.0f, 0.0f, 13.5f);//
      glEnd();
      glBegin(GL_TRIANGLES);
      glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
      glVertex3f(-2.0f, -0.2f, 10.0f);//
      glVertex3f(-1.0f, 0.5f, 10.0f);//
      glVertex3f(0.0f, 0.0f, 13.5f);//
      glEnd();

      //4
      //飞机驾驶舱
      glBegin(GL_TRIANGLES);
      glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
      glVertex3f(0.0f, 0.5f, 10.0f);//
      glVertex3f(0.25f, 1.5f, 9.25f);//
      glVertex3f(-0.25f, 1.5f, 9.25f);//
      glEnd();
      glBegin(GL_TRIANGLES);
      glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
      glVertex3f(0.0f, 0.5f, 10.0f);//
      glVertex3f(0.25f, 1.5f, 9.25f);//
      glVertex3f(1.0f, 0.5f, 9.0f);//
      glEnd();
      glBegin(GL_TRIANGLES);
      glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
      glVertex3f(0.0f, 0.5f, 10.0f);//
      glVertex3f(-0.25f, 1.5f, 9.25f);//
      glVertex3f(-1.0f, 0.5f, 9.0f);//
      glEnd();

      glBegin(GL_QUADS);
      glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
      glVertex3f(-1.0f, 0.5f, 9.0f);//
      glVertex3f(-0.25f, 1.5f, 9.25f);//
      glVertex3f(-0.25f, 1.5f, 7.75f);//
      glVertex3f(-1.0f, 0.5f, 8.0f);//
      glEnd();
      glBegin(GL_QUADS);
      glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
      glVertex3f(0.25f, 1.5f, 9.25f);//
      glVertex3f(-0.25f, 1.5f, 9.25f);//
      glVertex3f(-0.25f, 1.5f, 7.75f);//
      glVertex3f(0.25f, 1.5f, 7.75f);//
      glEnd();
      glBegin(GL_QUADS);
      glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
      glVertex3f(1.0f, 0.5f, 9.0f);//
      glVertex3f(0.25f, 1.5f, 9.25f);//
      glVertex3f(0.25f, 1.5f, 7.75f);//
      glVertex3f(1.0f, 0.5f, 8.0f);//
      glEnd();

      glBegin(GL_TRIANGLES);
      glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
      glVertex3f(0.0f, 0.5f, 7.0f);//
      glVertex3f(0.25f, 1.5f, 7.75f);//
      glVertex3f(-0.25f, 1.5f, 7.75f);//
      glEnd();
      glBegin(GL_TRIANGLES);
      glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
      glVertex3f(0.0f, 0.5f, 7.0f);//
      glVertex3f(0.25f, 1.5f, 7.75f);//
      glVertex3f(1.0f, 0.5f, 8.0f);//
      glEnd();
      glBegin(GL_TRIANGLES);
      glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
      glVertex3f(0.0f, 0.5f, 7.0f);//
      glVertex3f(-0.25f, 1.5f, 7.75f);//
      glVertex3f(-1.0f, 0.5f, 8.0f);//
      glEnd();

      //5
      //机身
      glBegin(GL_QUADS);
      glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
      glVertex3f(-0.25f, 0.25f, 1.0f);//
      glVertex3f(0.25f, 0.25f, 1.0f);//
      glVertex3f(0.6f, 0.5f, 4.0f);//
      glVertex3f(-0.6f, 0.5f, 4.0f);//
      glEnd();
      glBegin(GL_QUADS);
      glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
      glVertex3f(0.6f, 0.5f, 4.0f);//
      glVertex3f(-0.6f, 0.5f, 4.0f);//
      glVertex3f(-1.0f, 0.5f, 10.0f);//
      glVertex3f(1.0f, 0.5f, 10.0f);//
      glEnd();

      glBegin(GL_QUADS);
      glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
      glVertex3f(-0.5f, -0.2f, -1.0f);//
      glVertex3f(-0.25f, 0.2f, 1.0f);//
      glVertex3f(0.25f, 0.2f, 1.0f);//
      glVertex3f(0.5f, -0.2f, -1.0f);//
      glEnd();
      glBegin(GL_POLYGON);
      glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
      glVertex3f(2.0f, -0.2f, 10.0f);//
      glVertex3f(1.0f, 0.5f, 10.0f);//
      glVertex3f(0.6f, 0.5f, 4.0f);//
      glVertex3f(0.25f, 0.2f, 1.0f);//
      glVertex3f(0.5f, -0.2f, -1.0f);//
      glVertex3f(0.5f, -0.2f, -1.0f);//
      glVertex3f(1.2f, -0.2f, 4.0f);//
      glEnd();
      glBegin(GL_POLYGON);
      glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
      glVertex3f(0.0f, 0.0f, 13.5f);//
      glVertex3f(-2.0f, -0.2f, 10.0f);//
      glVertex3f(-0.5f, -0.2f, -1.0f);//
      glVertex3f(0.5f, -0.2f, -1.0f);//
      glVertex3f(2.0f, -0.2f, 10.0f);//
      glEnd();

      glBegin(GL_POLYGON);
      glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
      glVertex3f(-2.0f, -0.2f, 10.0f);//
      glVertex3f(-1.0f, 0.5f, 10.0f);//
      glVertex3f(-0.6f, 0.5f, 4.0f);//
      glVertex3f(-0.25f, 0.2f, 1.0f);//
      glVertex3f(-0.5f, -0.2f, -1.0f);//
      glVertex3f(-0.5f, -0.2f, -1.0f);//
      glVertex3f(-1.2f, -0.2f, 4.0f);//
      glEnd();

      //6
      //飞机机翼
      glBegin(GL_POLYGON);
      glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red 
      glVertex3f(-1.75f, 0.0f, 10.0f);//
      glVertex3f(-5.0f, 0.0f, 7.5f);//
      glVertex3f(-5.0f, 0.25f, 4.1f);//
      glVertex3f(-1.0f, 0.25f, 4.0f);//
      glVertex3f(-1.75f, 0.0f, 10.0f);//
      glEnd();
      glBegin(GL_POLYGON);
      glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
      glVertex3f(-5.0f, 0.0f, 7.5f);//
      glVertex3f(-5.0f, 0.25f, 4.1f);//
      glVertex3f(-9.0f, 0.45f, 2.5f);//
      glVertex3f(-10.0f, 0.2f, 5.5f);//
      glEnd();
      glBegin(GL_POLYGON);

      glBegin(GL_POLYGON);
      glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red 
      glVertex3f(1.75f, 0.0f, 10.0f);//
      glVertex3f(5.0f, 0.0f, 7.5f);//
      glVertex3f(5.0f, 0.25f, 4.1f);//
      glVertex3f(1.0f, 0.25f, 4.0f);//
      glVertex3f(1.75f, 0.0f, 10.0f);//
      glEnd();
      glBegin(GL_POLYGON);
      glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
      glVertex3f(5.0f, 0.0f, 7.5f);//
      glVertex3f(5.0f, 0.25f, 4.1f);//
      glVertex3f(9.0f, 0.45f, 2.5f);//
      glVertex3f(10.0f, 0.2f, 5.5f);//
      glEnd();

      //7
      //飞机后翼
      glBegin(GL_POLYGON);
      glColor3f(1.0f, 1.0f, 0.0f);//颜色是黄色yellow
      glVertex3f(-0.5f, -0.25f, -1.0f);//
      glVertex3f(-0.55f, 0.1f, 1.0f);//
      glVertex3f(-2.75f, 0.1f, -0.25f);//
      glVertex3f(-2.5f, -0.25f, -1.1f);//
      glEnd();
      glBegin(GL_POLYGON);
      glColor3f(1.0f, 1.0f, 0.0f);//颜色是黄色yellow
      glVertex3f(0.5f, -0.25f, -1.0f);//
      glVertex3f(0.55f, 0.1f, 1.0f);//
      glVertex3f(2.75f, 0.1f, -0.25f);//
      glVertex3f(2.5f, -0.25f, -1.1f);//
      glEnd();

      /*
      绘制圆锥
      int N = 36; // 决定圆锥的平滑程度，
      float R = 1.0f; // 半径
      float H = 1.0f; // 高
      float PI = 3.141593f;
      glBegin(GL_TRIANGLE_FAN);
      glVertex3f(0.0f, H, 0.0f);
      for (int i = 0; i < N; i++)
       {
      glVertex3f(0.0f + R * cos(i * 2 * PI / N), 0.0f, 0.0f + R * sin(i * 2 * PI / N));
      glVertex3f(0.0f + R * cos((i + 1) * 2 * PI / N), 0.0f, 0.0f + R * sin(i * 2 * PI / N));
       }
      glEnd();

      glBegin(GL_TRIANGLE_FAN);
      glVertex3f(0.0f, 0.0f, 0.0f);
      for (int i = 0; i < N; i++)
       {
      glVertex3f(0.0f + R * cos(i * 2 * PI / N), 0.0f, 0.0f + R * sin(i * 2 * PI / N));
      glVertex3f(0.0f + R * cos((i + 1) * 2 * PI / N), 0.0f, 0.0f + R * sin(i * 2 * PI / N));
       }
      glEnd();
      */

      /*
      glBegin(GL_POLYGON); 多边形
      glBegin(GL_QUADS); 四边形
      glBegin(GL_TRIANGLES); 三角形
      glEnd(); 结束
      */
  }

//|____________________________________________________________________
//|
//| Function: DrawPropeller
//|
//! \param width       [in] Width  of the propeller.
//! \param length      [in] Length of the propeller.
//! \return None.
//!
//! Draws a propeller.
//|____________________________________________________________________

void DrawPropeller(const float width, const float length)  //左右边方螺旋桨
{
  float w = width/2;
  float l = length/2;
  
  /*螺旋桨部分
  可以旋转
  配合按键：
  r,Rotates right propeller 
  y,Rotates left propeller 
  t,Rotates top propeller 
  */

  glBegin(GL_TRIANGLES);
  // Propeller is blue
  glColor3f(-0.2f, 1.0f, 1.0f);
  glVertex3f(0.0f, 0.25f, -2.0f);
  glVertex3f(0.5f, 0.25f, -2.0f);
  glVertex3f(0.0f, 0.25f,  0.0f);
  glEnd();

  glBegin(GL_TRIANGLES);
  // Propeller is blue
  glColor3f(-0.2f, 1.0f, 1.0f);
  glVertex3f( 0.0f, 0.25f, 2.0f);
  glVertex3f(-0.5f, 0.25f, 2.0f);
  glVertex3f( 0.0f, 0.25f, 0.0f);
  glEnd();

  glBegin(GL_TRIANGLES);
  // Propeller is white
  glColor3f(1.0f, 1.0f, 1.0f);
  glVertex3f(2.0f, 0.25f, 0.0f);
  glVertex3f(2.0f, 0.25f, 0.5f);
  glVertex3f(0.0f, 0.25f, 0.0f);
  glEnd();

  glBegin(GL_TRIANGLES);
  // Propeller is white
  glColor3f(1.0f, 1.0f, 1.0f);
  glVertex3f(-2.0f, 0.25f, 0.0f);
  glVertex3f(-2.0f, 0.25f,-0.5f);
  glVertex3f( 0.0f, 0.25f, 0.0f);
  glEnd();

  /*
  glBegin(GL_TRIANGLES);
    // Propeller is blue
    glColor3f(-0.2f, 1.0f, 1.0f);
    glVertex3f(-5.0f, 0.25f, 3.5f);
	glVertex3f(-4.5f, 0.25f, 3.5f);
	glVertex3f(-5.0f, 0.25f, 5.5f);
  glEnd();

  glBegin(GL_TRIANGLES);
    // Propeller is blue
    glColor3f(-0.2f, 1.0f, 1.0f);
    glVertex3f(-5.0f, 0.25f, 7.5f);
    glVertex3f(-5.5f, 0.25f, 7.5f);
    glVertex3f(-5.0f, 0.25f, 5.5f);
  glEnd();

  glBegin(GL_TRIANGLES);
    // Propeller is white
    glColor3f(1.0f, 1.0f, 1.0f);
    glVertex3f(-7.0f, 0.25f, 5.0f);
    glVertex3f(-7.0f, 0.25f, 5.5f);
    glVertex3f(-5.0f, 0.25f, 5.5f);
  glEnd();

  glBegin(GL_TRIANGLES);
    // Propeller is white
    glColor3f(1.0f, 1.0f, 1.0f);
    glVertex3f(-3.0f, 0.25f, 6.0f);
    glVertex3f(-3.5f, 0.25f, 5.5f);
    glVertex3f(-5.0f, 0.25f, 5.5f);
  glEnd();
  */
}
void DrawPropeller2(const float width, const float length)  //前方螺旋桨 Front Propeller
{
    float w = width / 2;
    float l = length / 2;

    /*螺旋桨部分
    可以旋转
    配合按键：
    r,Rotates right propeller
    y,Rotates left propeller
    t,Rotates top propeller
    */

    glBegin(GL_TRIANGLES);
    // Propeller is blue
    glColor3f(-0.2f, 1.0f, 1.0f);
    glVertex3f(0.0f, 3.0f, 13.5f);
    glVertex3f(1.0f, 3.0f, 13.5f);
    glVertex3f(0.0f, 0.0f, 13.5f);
    glEnd();

    glBegin(GL_TRIANGLES);
    // Propeller is blue
    glColor3f(-0.2f, 1.0f, 1.0f);
    glVertex3f(-1.0f,  -3.0f, 13.5f);
    glVertex3f( 0.0f, -3.0f, 13.5f);
    glVertex3f( 0.0f, 0.0f, 13.5f);
    glEnd();

    glBegin(GL_TRIANGLES);
    // Propeller is white
    glColor3f(0.8f, 1.0f, 1.0f);
    glVertex3f(3.0f,  0.0f, 13.5f);
    glVertex3f(3.0f, -1.0f, 13.5f);
    glVertex3f(0.0f, 0.0f, 13.5f);
    glEnd();

    glBegin(GL_TRIANGLES);
    // Propeller is white
    glColor3f(0.8f, 1.0f, 1.0f);
    glVertex3f(-3.0f, 0.0f, 13.5f);
    glVertex3f(-3.0f, 1.0f, 13.5f);
    glVertex3f( 0.0f, 0.0f, 13.5f);
    glEnd();

    /* //错误的坐标
    * glBegin(GL_TRIANGLES);
    // Propeller is blue
    glColor3f(-0.2f, 1.0f, 1.0f);
    glVertex3f(1.8f, 3.0f, 13.5f);
    glVertex3f(0.8f, 3.0f, 13.5f);
    glVertex3f(0.8f, 0.0f, 13.5f);
    glEnd();

    glBegin(GL_TRIANGLES);
    // Propeller is blue
    glColor3f(-0.2f, 1.0f, 1.0f);
    glVertex3f(0.8f,  -3.0f, 13.5f);
    glVertex3f(-0.2f, -3.0f, 13.5f);
    glVertex3f(0.8f, 0.0f, 13.5f);
    glEnd();

    glBegin(GL_TRIANGLES);
    // Propeller is white
    glColor3f(0.8f, 1.0f, 1.0f);
    glVertex3f(3.8f,  0.0f, 13.5f);
    glVertex3f(3.8f, -1.0f, 13.5f);
    glVertex3f(0.8f, 0.0f, 13.5f);
    glEnd();

    glBegin(GL_TRIANGLES);
    // Propeller is white
    glColor3f(0.8f, 1.0f, 1.0f);
    glVertex3f(-2.2f, 0.0f, 13.5f);
    glVertex3f(-2.2f, 1.0f, 13.5f);
    glVertex3f(0.8f, 0.0f, 13.5f);
    glEnd();
    */
}
void SubSubDrawPropeller(const float width, const float length)  //螺旋桨的辅组件
{
    float w = width / 2;
    float l = length / 2;

    /*螺旋桨部分
    可以旋转
    配合按键：
    r,Rotates right propeller
    y,Rotates left propeller
    t,Rotates top propeller
    */

    glBegin(GL_POLYGON);
    // Propeller is yellow

    glBegin(GL_TRIANGLES);
    // Propeller is yellow
    glColor3f(1.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 15.5f);
    glVertex3f(0.5f, 1 * sin(60), 13.5f);
    glVertex3f(-0.5f, 1 * sin(60), 13.5f);

    glBegin(GL_TRIANGLES);
    // Propeller is yellow
    glColor3f(1.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 15.5f);
    glVertex3f(0.5f, -1 * sin(60), 13.5f);
    glVertex3f(-0.5f, -1 * sin(60), 13.5f);
    glEnd();

    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(0.0f, 0.0f, 15.5f);
    glVertex3f(-1.0f, 0.0f, 13.5f);
    glVertex3f(-0.5f, 1 * sin(60), 13.5f);
    glEnd();

    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(0.0f, 0.0f, 15.5f);
    glVertex3f(1.0f, 0.0f, 13.5f);
    glVertex3f(0.5f, -1 * sin(60), 13.5f);
    glEnd();

    glBegin(GL_TRIANGLES);
    glColor3f(1.0f, 1.0f, 1.0f);//颜色是白white
    glVertex3f(0.0f, 0.0f, 15.5f);
    glVertex3f(1.0f, 0.0f, 13.5f);
    glVertex3f(0.5f, 1 * sin(60), 13.5f);
    glEnd();

    glBegin(GL_TRIANGLES);
    glColor3f(1.0f, 1.0f, 1.0f);//颜色是白white
    glVertex3f(0.0f, 0.0f, 15.5f);
    glVertex3f(-1.0f, 0.0f, 13.5f);
    glVertex3f(-0.5f, -1 * sin(60), 13.5f);
    glEnd();
}
//|____________________________________________________________________
//|
//| Function: main
//|
//! \param None.
//! \return None.
//!
//! Program entry point
//|____________________________________________________________________

int main(int argc, char **argv)
{ 
  InitTransforms();

  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);     // Uses GLUT_DOUBLE to enable double buffering
  glutInitWindowSize(w_width, w_height);
  
  glutCreateWindow("Plane Episode 2");

  glutDisplayFunc(DisplayFunc);
  glutKeyboardFunc(KeyboardFunc);
  glutMouseFunc(MouseFunc);
  glutMotionFunc(MotionFunc);
  glutReshapeFunc(ReshapeFunc);
  
  InitGL();

  glutMainLoop();

  return 0;
}