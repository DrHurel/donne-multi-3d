// -------------------------------------------
// gMini : a minimal OpenGL/GLUT application
// for 3D graphics.
// Copyright (C) 2006-2008 Tamy Boubekeur
// All rights reserved.
// -------------------------------------------

// -------------------------------------------
// Disclaimer: this code is dirty in the
// meaning that there is no attention paid to
// proper class attribute access, memory
// management or optimisation of any kind. It
// is designed for quick-and-dirty testing
// purpose.
// -------------------------------------------

#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Camera.h"
#include "Vec3.h"
#include <GL/glut.h>

enum class DisplayMode { WIRE = 0, SOLID = 1, LIGHTED_WIRE = 2, LIGHTED = 3 };

struct Triangle {

  std::array<unsigned int, 3> v = std::array<unsigned int, 3>();

  inline Triangle() { v[0] = v[1] = v[2] = 0; }
  inline Triangle(const Triangle &t) {
    v[0] = t.v[0];
    v[1] = t.v[1];
    v[2] = t.v[2];
  }
  inline Triangle(unsigned int v0, unsigned int v1, unsigned int v2) {
    v[0] = v0;
    v[1] = v1;
    v[2] = v2;
  }

  ~Triangle() = default;
  Triangle &operator=(const Triangle &) = delete;
};

struct Mesh {
  std::vector<Vec3> vertices;
  std::vector<Vec3> normals;
  std::vector<Triangle> triangles;
};

Mesh mesh;

// Mesh to generate
Mesh unit_sphere;

bool display_normals;
bool display_loaded_mesh;
bool display_unit_sphere;
DisplayMode displayMode;

// -------------------------------------------
// OpenGL/GLUT application code.
// -------------------------------------------

static GLint window;
const static unsigned int SCREENWIDTH = 1600;
const static unsigned int SCREENHEIGHT = 900;
static Camera camera;
static bool mouseRotatePressed = false;
static bool mouseMovePressed = false;
static bool mouseZoomPressed = false;
static int lastX = 0;
static int lastY = 0;
static int lastZoom = 0;
static bool fullScreen = false;
static int nX = 20;
static int nY = 20;

// To complete
void setUnitSphere(Mesh &o_mesh) {

  o_mesh.vertices.clear();
  o_mesh.normals.clear();
  o_mesh.triangles.clear();
  for (int i = 0; i <= nX; ++i) {
    for (int j = 0; j <= nY; ++j) {
      auto x = static_cast<float>(cos(2 * M_PI * i / nX) *
                                  cos((M_PI * j / nY) - M_PI_2));
      auto y = static_cast<float>(sin(2 * M_PI * i / nX) *
                                  cos((M_PI * j / nY) - M_PI_2));
      auto z = static_cast<float>(sin(M_PI * j / nY - M_PI_2));
      o_mesh.vertices.push_back(Vec3(x, y, z));
      o_mesh.normals.push_back(Vec3(x, y, z));
    }
  }

  for (unsigned int i = 0; i < o_mesh.vertices.size() - nX - 1; ++i) {
    o_mesh.triangles.push_back(Triangle(i, i + nX, i + nX + 1));
    o_mesh.triangles.push_back(Triangle(i, i + nX + 1, i + 1));
  }
}

void setUnitConne(Mesh &o_mesh, float h = 1.f) {

  o_mesh.vertices.clear();
  o_mesh.normals.clear();
  o_mesh.triangles.clear();

  o_mesh.vertices.push_back(Vec3(0, 0, h));
  o_mesh.normals.push_back(Vec3(0, 0, h));
  o_mesh.vertices.push_back(Vec3(0, 0, 0));
  o_mesh.normals.push_back(Vec3(0, 0, -h));
  for (int i = 0; i <= nX; ++i) {
    for (int j = 0; j <= nY; ++j) {
      auto x = static_cast<float>(cos(2 * M_PI * i / nX));

      auto y = static_cast<float>(sin(2 * M_PI * i / nX));

      o_mesh.vertices.push_back(Vec3(x, y, h));
      o_mesh.normals.push_back(Vec3(x, y, h));
    }
  }

  for (unsigned int i = 2; i < o_mesh.vertices.size() - 1; ++i) {
    o_mesh.triangles.push_back(Triangle(1, i, i + 1));
    o_mesh.triangles.push_back(Triangle(0, i, i + 1));
  }
}

bool saveOFF(const std::string &filename, std::vector<Vec3> &i_vertices,
             std::vector<Vec3> &i_normals, std::vector<Triangle> &i_triangles,
             bool save_normals = true) {
  std::ofstream myfile;
  myfile.open(filename.c_str());
  if (!myfile.is_open()) {
    std::cout << filename << " cannot be opened" << std::endl;
    return false;
  }

  std::vector<Vec3>::size_type n_vertices = i_vertices.size();
  std::vector<Vec3>::size_type n_triangles = i_triangles.size();
  myfile << n_vertices << " " << n_triangles << " 0" << std::endl;

  for (unsigned int v = 0; v < n_vertices; ++v) {
    myfile << i_vertices[v][0] << " " << i_vertices[v][1] << " "
           << i_vertices[v][2] << " ";
    if (save_normals)
      myfile << i_normals[v][0] << " " << i_normals[v][1] << " "
             << i_normals[v][2] << std::endl;
    else
      myfile << std::endl;
  }
  for (unsigned int f = 0; f < n_triangles; ++f) {
    myfile << 3 << " " << i_triangles[f].v[0] << " " << i_triangles[f].v[1]
           << " " << i_triangles[f].v[2];
    myfile << std::endl;
  }
  myfile.close();
  return true;
}

void openOFF(std::string const &filename, std::vector<Vec3> &o_vertices,
             std::vector<Vec3> &o_normals, std::vector<Triangle> &o_triangles,
             bool load_normals = true) {
  std::ifstream myfile;
  myfile.open(filename.c_str());
  if (!myfile.is_open()) {
    std::cout << filename << " cannot be opened" << std::endl;
    return;
  }

  std::string magic_s;

  myfile >> magic_s;

  if (magic_s != "OFF") {
    std::cout << magic_s << " != OFF :   We handle ONLY *.off files."
              << std::endl;
    myfile.close();
    exit(1);
  }

  int n_vertices;
  int n_faces;
  int dummy_int;
  myfile >> n_vertices >> n_faces >> dummy_int;

  o_vertices.clear();
  o_normals.clear();

  for (int v = 0; v < n_vertices; ++v) {
    float x;
    float y;
    float z;

    myfile >> x >> y >> z;
    o_vertices.push_back(Vec3(x, y, z));

    if (load_normals) {
      myfile >> x >> y >> z;
      o_normals.push_back(Vec3(x, y, z));
    }
  }

  o_triangles.clear();
  for (int f = 0; f < n_faces; ++f) {
    int n_vertices_on_face;
    myfile >> n_vertices_on_face;

    if (n_vertices_on_face == 3) {
      unsigned int _v1;
      unsigned int _v2;
      unsigned int _v3;
      myfile >> _v1 >> _v2 >> _v3;

      o_triangles.push_back(Triangle(_v1, _v2, _v3));
    } else if (n_vertices_on_face == 4) {
      unsigned int _v1;
      unsigned int _v2;
      unsigned int _v3;
      unsigned int _v4;
      myfile >> _v1 >> _v2 >> _v3 >> _v4;

      o_triangles.push_back(Triangle(_v1, _v2, _v3));
      o_triangles.push_back(Triangle(_v1, _v3, _v4));
    } else {
      std::cout << "We handle ONLY *.off files with 3 or 4 vertices per face"
                << std::endl;
      myfile.close();
      exit(1);
    }
  }
}

// ------------------------------------

void initLight() {
  std::array<GLfloat, 4> light_position1 = {22.0f, 16.0f, 50.0f, 0.0f};
  std::array<GLfloat, 4> direction1 = {-52.0f, -16.0f, -50.0f};
  std::array<GLfloat, 4> color1 = {1.0f, 1.0f, 1.0f, 1.0f};
  std::array<GLfloat, 4> ambient = {0.3f, 0.3f, 0.3f, 0.5f};

  glLightfv(GL_LIGHT1, GL_POSITION, light_position1.data());
  glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, direction1.data());
  glLightfv(GL_LIGHT1, GL_DIFFUSE, color1.data());
  glLightfv(GL_LIGHT1, GL_SPECULAR, color1.data());
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient.data());
  glEnable(GL_LIGHT1);
  glEnable(GL_LIGHTING);
}

void init() {
  camera.resize(SCREENWIDTH, SCREENHEIGHT);
  initLight();
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
  glDepthFunc(GL_LESS);
  glEnable(GL_DEPTH_TEST);
  glClearColor(0.2f, 0.2f, 0.3f, 1.0f);
  glEnable(GL_COLOR_MATERIAL);

  displayMode = DisplayMode::LIGHTED;
  display_normals = false;
  display_unit_sphere = false;
  display_loaded_mesh = true;

  glLineWidth(1.);
  glPointSize(4.);
}

// -------------------------------------------
// rendering.
// -------------------------------------------

void drawVector(Vec3 const &i_from, Vec3 const &i_to) {

  glBegin(GL_LINES);
  glVertex3f(i_from[0], i_from[1], i_from[2]);
  glVertex3f(i_to[0], i_to[1], i_to[2]);
  glEnd();
}

void drawVertices(Mesh const &i_mesh) {
  glBegin(GL_POINTS); // Fonction OpenGL de dessin de points
  for (const auto &p : i_mesh.vertices) {
    glVertex3f(p[0], p[1], p[2]);
  }
  glEnd();
}

void drawTriangleMesh(Mesh const &i_mesh) {

  if (!i_mesh.triangles.empty()) {
    if (!i_mesh.normals.empty()) {
      // Fonction de dessin en utilisant les normales au sommet
      glBegin(GL_TRIANGLES); // Fonction OpenGL de dessin de triangles
      for (const auto &triangle : i_mesh.triangles) {
        Vec3 p0 = i_mesh.vertices[triangle.v.at(0)];
        Vec3 n0 = i_mesh.normals[triangle.v.at(0)];

        Vec3 p1 = i_mesh.vertices[triangle.v.at(1)];
        Vec3 n1 = i_mesh.normals[triangle.v.at(1)];

        Vec3 p2 = i_mesh.vertices[triangle.v.at(2)];
        Vec3 n2 = i_mesh.normals[triangle.v.at(2)];

        glNormal3f(n0[0], n0[1], n0[2]);
        glVertex3f(p0[0], p0[1], p0[2]);
        glNormal3f(n1[0], n1[1], n1[2]);
        glVertex3f(p1[0], p1[1], p1[2]);
        glNormal3f(n2[0], n2[1], n2[2]);
        glVertex3f(p2[0], p2[1], p2[2]);
      }
      glEnd();
    } else {
      // Fonction de dessin en utilisant sans normales
      glBegin(GL_TRIANGLES); // Fonction OpenGL de dessin de triangles
      for (const auto &triangle : i_mesh.triangles) {
        Vec3 p0 = i_mesh.vertices[triangle.v[0]];
        Vec3 p1 = i_mesh.vertices[triangle.v[1]];
        Vec3 p2 = i_mesh.vertices[triangle.v[2]];

        // Dessin des trois sommets formant le triangle
        glVertex3f(p0[0], p0[1], p0[2]);
        glVertex3f(p1[0], p1[1], p1[2]);
        glVertex3f(p2[0], p2[1], p2[2]);
      }
      glEnd();
    }
  } else {
    drawVertices(i_mesh);
  }

  if (display_normals) {

    glColor3f(1.f, 0.f, 0.f);
    for (unsigned int pIt = 0; pIt < i_mesh.normals.size(); ++pIt) {
      Vec3 to = i_mesh.vertices[pIt] + 0.02f * i_mesh.normals[pIt];
      drawVector(i_mesh.vertices[pIt], to);
    }
  }
}

void draw() {

  if (displayMode == DisplayMode::LIGHTED ||
      displayMode == DisplayMode::LIGHTED_WIRE) {

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_LIGHTING);

  } else if (displayMode == DisplayMode::WIRE) {

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDisable(GL_LIGHTING);

  } else if (displayMode == DisplayMode::SOLID) {
    glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }

  if (display_unit_sphere) {
    glColor3f(0.8f, 1.f, 0.8f);
    drawTriangleMesh(unit_sphere);
  }

  if (display_loaded_mesh) {
    glColor3f(0.8f, 0.8f, 1.f);
    drawTriangleMesh(mesh);
  }

  if (displayMode == DisplayMode::SOLID ||
      displayMode == DisplayMode::LIGHTED_WIRE) {
    glEnable(GL_POLYGON_OFFSET_LINE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(1.0f);
    glPolygonOffset(-2.0, 1.0);

    glColor3f(0., 0., 0.);
    if (display_unit_sphere)
      drawTriangleMesh(unit_sphere);

    if (display_loaded_mesh)
      drawTriangleMesh(mesh);

    glDisable(GL_POLYGON_OFFSET_LINE);
    glEnable(GL_LIGHTING);
  }
}

void changeDisplayMode() {
  if (displayMode == DisplayMode::LIGHTED)
    displayMode = DisplayMode::LIGHTED_WIRE;
  else if (displayMode == DisplayMode::LIGHTED_WIRE)
    displayMode = DisplayMode::SOLID;
  else if (displayMode == DisplayMode::SOLID)
    displayMode = DisplayMode::WIRE;
  else
    displayMode = DisplayMode::LIGHTED;
}

void display() {
  glLoadIdentity();
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  camera.apply();
  draw();
  glFlush();
  glutSwapBuffers();
}

void idle() { glutPostRedisplay(); }

void key(unsigned char keyPressed, [[maybe_unused]] int x,
         [[maybe_unused]] int y) {
  switch (keyPressed) {
  case 'f':
    if (fullScreen == true) {
      glutReshapeWindow(SCREENWIDTH, SCREENHEIGHT);
      fullScreen = false;
    } else {
      glutFullScreen();
      fullScreen = true;
    }
    break;

  case 'w': // Change le mode d'affichage
    changeDisplayMode();
    break;

  case 'n': // Press n key to display normals
    display_normals = !display_normals;
    break;

  case '1': // Toggle loaded mesh display
    display_loaded_mesh = !display_loaded_mesh;
    break;

  case '2': // Toggle unit sphere mesh display
    display_unit_sphere = !display_unit_sphere;
    break;
  case '-':
    if (nX > 2) {
      --nX;
      --nY;
    }
    setUnitSphere(unit_sphere);
    break;
  case '+':
    ++nX;
    ++nY;
    setUnitSphere(unit_sphere);
    break;
  case 'c':
    setUnitConne(unit_sphere);
    break;

  default:
    break;
  }
  idle();
}

void mouse(int button, int state, int x, int y) {
  if (state == GLUT_UP) {
    mouseMovePressed = false;
    mouseRotatePressed = false;
    mouseZoomPressed = false;
  } else {
    if (button == GLUT_LEFT_BUTTON) {
      camera.beginRotate(x, y);
      mouseMovePressed = false;
      mouseRotatePressed = true;
      mouseZoomPressed = false;
    } else if (button == GLUT_RIGHT_BUTTON) {
      lastX = x;
      lastY = y;
      mouseMovePressed = true;
      mouseRotatePressed = false;
      mouseZoomPressed = false;
    } else if (button == GLUT_MIDDLE_BUTTON && mouseZoomPressed == false) {
      lastZoom = y;
      mouseMovePressed = false;
      mouseRotatePressed = false;
      mouseZoomPressed = true;
    }
  }
  idle();
}

void motion(int x, int y) {
  if (mouseRotatePressed == true) {
    camera.rotate(x, y);
  } else if (mouseMovePressed == true) {
    camera.move(static_cast<float>(x - lastX) / SCREENWIDTH,
                static_cast<float>(lastY - y) / SCREENHEIGHT, 0.0);
    lastX = x;
    lastY = y;
  } else if (mouseZoomPressed == true) {
    camera.zoom(static_cast<float>(y - lastZoom) / SCREENHEIGHT);
    lastZoom = y;
  }
}

void reshape(int w, int h) { camera.resize(w, h); }

int main(int argc, char **argv) {
  if (argc > 2) {
    exit(EXIT_FAILURE);
  }
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
  glutInitWindowSize(SCREENWIDTH, SCREENHEIGHT);
  window = glutCreateWindow("TP HAI714I");

  init();
  glutIdleFunc(idle);
  glutDisplayFunc(display);
  glutKeyboardFunc(key);
  glutReshapeFunc(reshape);
  glutMotionFunc(motion);
  glutMouseFunc(mouse);
  key('?', 0, 0);

  // Unit sphere mesh loaded with precomputed normals
  openOFF("data/unit_sphere_n.off", mesh.vertices, mesh.normals,
          mesh.triangles);

  // Uncomment to see other meshes
  // openOFF("data/elephant_n.off", mesh.vertices, mesh.normals,
  // mesh.triangles);

  setUnitSphere(unit_sphere);

  glutMainLoop();
  return EXIT_SUCCESS;
}
