// --------------------------------------------------------------------------
// gMini,
// a minimal Glut/OpenGL app to extend
//
// Copyright(C) 2007-2009
// Tamy Boubekeur
//
// All rights reserved.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License (http://www.gnu.org/licenses/gpl.txt)
// for more details.
//
// --------------------------------------------------------------------------

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>
#include <cstdlib>

#define GLEW_STATIC 1
#include <GL/glew.h>
#include <GL/glut.h>

#include "src/Shader.h"
#include "src/Vec3D.h"
#include "src/Vertex.h"
#include "src/Triangle.h"
#include "src/Mesh.h"
#include "src/Camera.h"

using namespace std;

class PhongShader : public Shader {
public:
    PhongShader () { init ("shader.vert", "shader.frag"); }
    inline virtual ~PhongShader () {}

    void setAmbientRef (float s) {
        glUniform1fARB (ambientRefLocation, s);
    }

    void setDiffuseRef (float s) {
        glUniform1fARB (diffuseRefLocation, s);
    }

    void setSpecularRef (float s) {
        glUniform1fARB (specularRefLocation, s);
    }

    void setShininess (float s) {
        glUniform1fARB (shininessLocation, s);
    }

    void setLevels (int l) {
        glUniform1iARB (levelsLocation, l);
    }

private:
    void init (const std::string & vertexShaderFilename,
               const std::string & fragmentShaderFilename) {
        loadFromFile (vertexShaderFilename, fragmentShaderFilename);
        bind ();
        ambientRefLocation = getUniLoc ("ambientRef");
        diffuseRefLocation = getUniLoc ("diffuseRef");
        specularRefLocation = getUniLoc ("specularRef");
        shininessLocation = getUniLoc ("shininess");
        levelsLocation = getUniLoc ("levels");
    }
    GLint ambientRefLocation;
    GLint diffuseRefLocation;
    GLint specularRefLocation;
    GLint shininessLocation;
    GLint levelsLocation;
};

static GLint window;
static unsigned int SCREENWIDTH = 1024;
static unsigned int SCREENHEIGHT = 768;
static Camera camera;
static bool mouseRotatePressed = false;
static bool mouseMovePressed = false;
static bool mouseZoomPressed = false;
static int lastX=0, lastY=0, lastZoom=0;
static unsigned int FPS = 0;
static bool fullScreen = false;

static PhongShader * phongShader;

static Mesh current_mesh;
static Mesh diabolo_mesh;
static Mesh diabolo_mesh_offset;
static Mesh diabolo_mesh_offset_1;
static Mesh plan_mesh;
static Mesh camel;

std::vector<Mesh *> meshes;
static GLuint glID;

static int levels = 4;
static float ambientRef = 0.2f;
static float diffuseRef = 0.6f;
static float specularRef = 0.4f;
static float shininess = 30.0f;
static float angle = 0.f;
static float offset = 2.f;

// Je cree une variable globale pour controler la resolution de mon diabolo
static int numberOfVerticesOnCircles = 10;



typedef enum {Wire, Phong, Solid} RenderingMode;
typedef enum {Modelisation, Rendu, Animation, Exemples} Exercice;
static RenderingMode mode = Phong;
static Exercice exercice = Modelisation;
void initGLList ();


bool saveOFF( const std::string & filename ,
              Mesh * i_mesh ,
              bool save_normals = true ) {
    std::ofstream myfile;
    myfile.open(filename.c_str());
    if (!myfile.is_open()) {
        std::cout << filename << " cannot be opened" << std::endl;
        return false;
    }

    std::vector< Vertex > & i_vertices = i_mesh->getVertices();
    std::vector< Triangle > & i_triangles = i_mesh->getTriangles();

    myfile << "OFF" << std::endl ;

    unsigned int n_vertices = i_vertices.size() , n_triangles = i_triangles.size();
    myfile << n_vertices << " " << n_triangles << " 0" << std::endl;

    for( unsigned int v = 0 ; v < n_vertices ; ++v ) {
        myfile << i_vertices[v].position[0] << " " << i_vertices[v].position[1] << " " << i_vertices[v].position[2] << " ";
        if (save_normals) myfile << i_vertices[v].normal[0] << " " << i_vertices[v].normal[1] << " " << i_vertices[v].normal[2] << std::endl;
        else myfile << std::endl;
    }
    for( unsigned int f = 0 ; f < n_triangles ; ++f ) {
        myfile << 3 << " " << i_triangles[f][0] << " " << i_triangles[f][1] << " " << i_triangles[f][2];
        myfile << std::endl;
    }
    myfile.close();
    return true;
}

void openOFF (const std::string filename, Mesh &mesh, unsigned int normWeight) {
    vector<Vertex> V;
    vector<Triangle> T;

    ifstream in (filename.c_str ());
    if (!in)
        exit (EXIT_FAILURE);
    string offString;
    unsigned int sizeV, sizeT, tmp;
    in >> offString >> sizeV >> sizeT >> tmp;
    for (unsigned int i = 0; i < sizeV; i++) {
        Vec3 p;
        in >> p;
        V.push_back (Vertex (p));
    }
    int s;
    for (unsigned int i = 0; i < sizeT; i++) {
        in >> s;
        unsigned int v[3];
        for (unsigned int j = 0; j < 3; j++)
            in >> v[j];
        T.push_back (Triangle (v[0], v[1], v[2]));
    }
    in.close ();

    Vec3 center;
    float radius;
    // Vertex::scaleToUnitBox (V, center, radius);
    mesh = Mesh (V, T);
    mesh.recomputeSmoothVertexNormals (normWeight);

}

inline void glVertexVec3Df (const Vec3 & v) {
    glVertex3f (v[0], v[1], v[2]);
}

inline void glNormalVec3Df (const Vec3 & n) {
    glNormal3f (n[0], n[1], n[2]);
}

inline void glDrawPoint (const Vec3 & pos, const Vec3 & normal) {
    glNormalVec3Df (normal);
    glVertexVec3Df (pos);
}

inline void glDrawPoint (const Vertex & v) {
    glDrawPoint (v.getPosition (), v.getNormal ());
}

//Exercice 1
//Fonction à completer pour obtenir un dibolo constitué de 2 cones collés
//soit en construisant 2 cones inversés soit en faisant un cylindre avec un rayon variable
void buildModel(Mesh & o_mesh, unsigned int n=10, float radius=0.5, float height=1., unsigned int ny = 3 ){
    std::vector<Vertex> & V = o_mesh.getVertices(); //Calculer les positions des sommets et les mettre dans le vecteur V
    std::vector<Triangle> & T = o_mesh.getTriangles(); //Discrétiser la forme et mettre les triangle dans le vecteur T

    V.clear();
    T.clear();

    //A completer :
    // Utiliser l'équation d'un cercle pour les bases de la forme
    //Vous utiliserez le rayon radius et la forme doit avoir une hauteur de height et être centrée en zéro

    // D'abord, le cercle de la partie basse du diabolo
    // On crée n points, donc avec les indices de 0 à n-1
    for (int i = 0; i < n; i++) {
        // Calcul de l'angle du cercle sur lequel on va placer un nouveau point
        float a = (float(i) / float(n - 1)) * 2.f * M_PI;
        // Equation du cercle, classique
        float x = radius * cos(a);
        float y = radius * sin(a);

        // Ajout du vertex dans le maillage
        Vec3 pos1(x, -height / 2.f, y);

        V.push_back(pos1);

        // Ici, je n'ai pas besoin de connaitre la position de tous les points pour créer la topologie!
        // Avec un peu de logique mise sur papier, je sais que je vais faire 1 triangle avec un point 
        // au centre de mon cercle (pour créer un disque) et un triangle avec un point au centre de
        // mon maillage (pour créer un cone)
        // En fin de fonction, je décris pourquoi je sais que ces indices sont respectivement 2n et 2n+2
        if (i < n - 1) { // J'ignore le premier point (J'ai "n" sommets, mais seulement "n-1" segments entre les sommets)
            Triangle tri1(i, (i + 1), 2 * n + 0);
            Triangle tri2((i + 1), i, 2 * n + 2);
            // J'insère mes triangle dans le maillage
            T.push_back(tri1);
            T.push_back(tri2);
        }
    }
    
    // Exactement pareil pour le cercle de la partie haute
    // Les indices vont de "n" à "2n - 1"
    for (int i = 0; i < n; i++) {
        float a = (float(i) / float(n - 1)) * 2.f * M_PI;
        float x = radius * cos(a);
        float y = radius * sin(a);

        Vec3 pos1(x, height / 2.f, y);

        V.push_back(pos1);
        

        if (i < n - 1) {
            Triangle tri1((i + 1) + n, i + n, 2 * n + 1);
            Triangle tri2(i + n, (i + 1) + n, 2 * n + 2);

            T.push_back(tri1);
            T.push_back(tri2);
        }
    }

    // Un point au centre bas sur lequel tous les points du cercle bas vont se connecter
    // pour créer un disque. On sait qu'il sera à l'indice "2n" (ou "2n+0")
    V.push_back(Vec3(0, -height / 2.f, 0));
    // Un point au centre haut sur lequel tous les points du cercle haut vont se connecter
    // pour créer un disque. On sait qu'il sera à l'indice "2n+1"
    V.push_back(Vec3(0, height / 2.f, 0));
    // Le point central du diabolo, sur lequel les deux cercles se connectent pour créer des cones (indice 2n+2)
    V.push_back(Vec3(0, 0, 0));

    // Pour info, je ne calcule pas la normale des points. La raison est qu'aucun sommet n'a de normale
    // définie mathématiquement à part les sommets au centre des disques.
    // Si on souhaite renseigner les normales, il faudrait créer chaque vertex 2 fois : une fois avec la 
    // normale de la pointe du cone, et une fois pour le disque.
    // Le sommet central aux coordonnes (0, 0, 0) est complètement indéfini car notre maillage
    // est "non-manifold" (ou "non-variété")
}

//Construction du plan de la scène
void buildPlan(Mesh & o_mesh, unsigned int nx=30, unsigned int ny = 30, Vec3 origin = Vec3 (-25.,0.,-25.), Vec3 right = Vec3 (25.,0.,0.),  Vec3 up = Vec3(0.,0.,25.) ){

    std::vector<Vertex> & V = o_mesh.getVertices();
    std::vector<Triangle> & T = o_mesh.getTriangles();

    V.clear();
    T.clear();

    float sizeX = right.getLength();
    float sizeY = up.getLength();


    float stepX = sizeX/float (nx-1);
    float stepY = sizeY/float (ny-1);

    for( unsigned int j = 0 ; j < ny ; j ++ ){
        for( unsigned int i = 0 ; i < nx ; i ++ ){


            Vec3 position = origin+i*stepX*right +j*stepY*up;
            Vec3 normal = Vec3::crossProduct(up,right);
            normal.normalize();
            V.push_back( Vertex(position, normal)  );
        }
    }

    for( unsigned int j = 0 ; j < ny-1 ; j ++ ){
        for( unsigned int i = 0 ; i < nx-1 ; i ++ ){
            T.push_back(Triangle(j*nx+i, (j+1)*nx+i, j*nx+i+1));
            T.push_back(Triangle((j+1)*nx+i,(j+1)*nx+i+1,j*nx+i+1));
        }
    }


}

//Exercice 2 a completer
void updateAnimation (){
    // On va modifier la position des sommets de notre maillage à partir d'un modèle "statique"
    // (le maillage "diabolo_mesh"). Il est impératif que les deux maillages aient le meme nombre
    // de sommets. Dans le doute, maintenant que mon exercice 1 est fini, je vais l'utiliser ici.
    buildModel(diabolo_mesh, numberOfVerticesOnCircles);

    //Récupérer la position des sommets du maillage courant à mettre à jour
    vector<Vertex> & V = current_mesh.getVertices ();
    vector<Vertex> & V0 = diabolo_mesh.getVertices ();

    // Juste un vecteur de translation pour decaler mon modele un peu sur l'axe X
    Vec3 translation(offset, 0., 0.);

    // Mes matrices de rotation X, Y, Z que je vais combiner ensuite
    Mat3 Rx, Ry, Rz;

    Mat3 rotation = Mat3::Identity();

    // Je veux un quart de tour sur l'axe X.
    Rx = Mat3(1, 0, 0, 0, cos(M_PI / 2.f), -sin(M_PI / 2.f), 0, sin(M_PI / 2.f), cos(M_PI / 2.f));
    // En réalité, M_PI / 2 est constant, donc je peux le calculer "a la main" (cos(pi/2) = 0 et sin(pi/2) = 1)
    // Rx = Mat3(1, 0, 0, 0, 0, -1, 0, 1, 0);
    // Vous connaissez cette matrice :
    Rz = Mat3(cos(angle), sin(angle), 0, -sin(angle), cos(angle), 0, 0, 0, 1);
    // Si je ne veux pas faire de rotation, j'utilise la matrice identite
    Ry = Mat3::Identity();

    // Finalement, je combine en une seule matrice:
    rotation = Rx * Ry * Rz;
    // (Ca reduit considerablement le nombre de multiplication de matrices dans la boucle for)
    for(  unsigned int i = 0 ; i < V.size() ; i++ ){
        // Et maintenant, je peux appliquer une rotation et translation a tous mes sommets.
        // Retenez bien que appliquer rotation puis translation est different de translation puis rotation
        V[i].position = rotation * (V0[i].position + translation);
    }

    initGLList ();
}

void setShaderValues () {
    phongShader->setAmbientRef(ambientRef);
    phongShader->setDiffuseRef (diffuseRef);
    phongShader->setSpecularRef (specularRef);
    phongShader->setShininess (shininess);
    phongShader->setLevels(levels);
}

void drawMesh (Mesh * mesh, float flat = true ) {

    const vector<Vertex> & V = mesh->getVertices ();
    const vector<Triangle> & T = mesh->getTriangles();
    glBegin (GL_TRIANGLES);
    for (unsigned int i = 0; i < T.size (); i++) {
        const Triangle & t = T[i];
        if (flat) {
            Vec3 normal = Vec3::crossProduct (V[t.getVertex (1)].getPosition ()
                    - V[t.getVertex (0)].getPosition (),
                    V[t.getVertex (2)].getPosition ()
                    - V[t.getVertex (0)].getPosition ());
            normal.normalize ();
            glNormalVec3Df (normal);
        }
        for (unsigned int j = 0; j < 3; j++)
            if (!flat) {
                glNormalVec3Df (V[t.getVertex (j)].getNormal ());
                glVertexVec3Df (V[t.getVertex (j)].getPosition ());
            } else
                glVertexVec3Df (V[t.getVertex (j)].getPosition ());
    }
    glEnd ();
}

void drawSolidModel () {
    glEnable (GL_LIGHTING);
    glEnable (GL_COLOR_MATERIAL);
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
    glPolygonOffset (1.0, 1.0);
    glEnable (GL_POLYGON_OFFSET_FILL);
    glShadeModel (GL_FLAT);
    phongShader->bind ();
    for(unsigned int i = 0; i < meshes.size() ; i ++)
        drawMesh (meshes[i]);

    glPolygonMode (GL_FRONT, GL_LINE);
    glPolygonMode (GL_BACK, GL_FILL);
    glColor3f (0.0, 0.0, 0.0);
    for(unsigned int i = 0; i < meshes.size() ; i ++)
        drawMesh (meshes[i]);
    glDisable (GL_POLYGON_OFFSET_FILL);
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
    glDisable (GL_COLOR_MATERIAL);
    glDisable (GL_LIGHTING);
    glShadeModel (GL_SMOOTH);
}

void drawPhongModel () {
    glCallList (glID);
}

void initLights () {

    GLfloat light_position_0[4] = {0., 2., 0., 0};
    GLfloat light_position_1[4] = {0., -2., -0., 0};
    GLfloat light_position_2[4] = {-2., 1., -0.5, 0};

    GLfloat direction_0[3] = {0., -2., 0.};
    GLfloat direction_1[3] = {0., 2., 0.};
    GLfloat direction_2[3] = {2., -1., 0.5};

    GLfloat diffuse_color_0[4] = {0.8, 0.8, 0.8, 1};
    GLfloat diffuse_color_1[4] = {0.08, 0.29, 0.8, 1};
    GLfloat diffuse_color_2[4] = {0.8, 0.29, 0.03, 1};

    GLfloat specular_color_0[4] = {0.2, 0.0, 0.0, 1};
    GLfloat specular_color_1[4] = {0.0, 0.2, 0.0, 1};
    GLfloat specular_color_2[4] = {0.0, 0.0, 0.2, 1};


    GLfloat cut_off_0 = 45.f;
    GLfloat cut_off_1 = 45.f;
    GLfloat cut_off_2 = 45.f;

    GLfloat ambient[4] = {0.4f, 0.4f, 0.4f, 1.f};

    glLightfv (GL_LIGHT0, GL_POSITION, light_position_0);
    glLightfv (GL_LIGHT0, GL_SPOT_DIRECTION, direction_0);
    glLightfv (GL_LIGHT0, GL_DIFFUSE, diffuse_color_0);
    glLightfv (GL_LIGHT0, GL_SPECULAR, specular_color_0);
    glLightfv (GL_LIGHT0, GL_SPOT_CUTOFF, &cut_off_0);

    glLightfv (GL_LIGHT1, GL_POSITION, light_position_1);
    glLightfv (GL_LIGHT1, GL_SPOT_DIRECTION, direction_1);
    glLightfv (GL_LIGHT1, GL_DIFFUSE, diffuse_color_1);
    glLightfv (GL_LIGHT1, GL_SPECULAR, specular_color_1);
    glLightfv (GL_LIGHT1, GL_SPOT_CUTOFF, &cut_off_1);

    glLightfv (GL_LIGHT2, GL_POSITION, light_position_2);
    glLightfv (GL_LIGHT2, GL_SPOT_DIRECTION, direction_2);
    glLightfv (GL_LIGHT2, GL_DIFFUSE, diffuse_color_2);
    glLightfv (GL_LIGHT2, GL_SPECULAR, specular_color_2);
    glLightfv (GL_LIGHT2, GL_SPOT_CUTOFF, &cut_off_2);

    glLightModelfv (GL_LIGHT_MODEL_AMBIENT, ambient);

    glEnable (GL_LIGHTING);
}

void setSunriseLight () {
    glDisable (GL_LIGHT0);
    glDisable (GL_LIGHT1);
    glDisable (GL_LIGHT2);
}

void setSingleSpotLight () {
    glEnable (GL_LIGHT0);
    glDisable (GL_LIGHT1);
    glDisable (GL_LIGHT2);
}

void setDefaultMaterial () {
    GLfloat material_color[4] = {1.0, 1.0, 1., 1.0f };
    GLfloat material_specular[4] = {0.5, 0.5, 0.5, 1.0 };
    GLfloat material_ambient[4] = {1.0, 0.0, 0.0, 1.0};

    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, material_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material_color);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, material_ambient);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 128);

    glDisable (GL_COLOR_MATERIAL);
}

void initGLList () {
    glID = glGenLists (1);
    glNewList (glID, GL_COMPILE);
    for(unsigned int i = 0; i < meshes.size() ; i ++)
        drawMesh (meshes[i]);
    glEndList ();
}

void setExercice( int numero ){
    meshes.clear();

    buildModel(current_mesh, numberOfVerticesOnCircles);
    

    if( numero == 1 ) {
        exercice = Modelisation;
        meshes.push_back(&current_mesh);
    }
    if( numero == 2 ) {
        exercice = Animation;
        meshes.push_back(&current_mesh);
    }
    if( numero == 3 ) {
        exercice = Rendu;
        meshes.push_back(&plan_mesh);
        meshes.push_back(&current_mesh);
    }
    if( numero == 4 ) {
        exercice = Exemples;
        meshes.push_back(&diabolo_mesh_offset);
        meshes.push_back(&diabolo_mesh_offset_1);
        meshes.push_back(&current_mesh);
    }
    initGLList();
}

void init () {
    glewInit();
    if (glewGetExtension ("GL_ARB_vertex_shader")        != GL_TRUE ||
            glewGetExtension ("GL_ARB_shader_objects")       != GL_TRUE ||
            glewGetExtension ("GL_ARB_shading_language_100") != GL_TRUE) {
        cerr << "Driver does not support OpenGL Shading Language" << endl;
        exit (EXIT_FAILURE);
    }
    if (glewGetExtension ("GL_ARB_vertex_buffer_object") != GL_TRUE) {
        cerr << "Driver does not support Vertex Buffer Objects" << endl;
        exit (EXIT_FAILURE);
    }

    camera.resize (SCREENWIDTH, SCREENHEIGHT);
    camera.move(0.,-0.15,0.);

    glClearColor (0.8, 0.8, 0.8, 1.0);

    initLights ();
    setSunriseLight ();
    setDefaultMaterial ();


    openOFF(std::string("./data/diabolo.off"), diabolo_mesh, 0);
    openOFF(std::string("./data/diabolo_offset.off"), diabolo_mesh_offset, 0);
    openOFF(std::string("./data/diabolo_offset_1.off"), diabolo_mesh_offset_1, 0);


    buildPlan(plan_mesh);

    buildModel(current_mesh, numberOfVerticesOnCircles);

    meshes.push_back(&diabolo_mesh);

    setExercice(4);


    initGLList ();


    try {
        phongShader = new PhongShader;
        phongShader->bind ();
        setShaderValues ();
    } catch (ShaderException e) {
        cerr << e.getMessage () << endl;
        exit (EXIT_FAILURE);
    }
}

void clear () {
    delete phongShader;
    glDeleteLists (glID, 1);
}

void reshape(int w, int h) {
    camera.resize (w, h);
}

void display () {
    glLoadIdentity ();
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera.apply ();
    drawPhongModel ();
    glFlush ();
    glutSwapBuffers ();
}

void idle () {
    static float lastTime = glutGet ((GLenum)GLUT_ELAPSED_TIME);
    static unsigned int counter = 0;
    counter++;
    float currentTime = glutGet ((GLenum)GLUT_ELAPSED_TIME);
    if (currentTime - lastTime >= 1000.0f) {
        FPS = counter;
        counter = 0;
        static char FPSstr [128];
        unsigned int numOfTriangles = current_mesh.getTriangles ().size ();
        if (mode == Solid)
            sprintf (FPSstr, "HAI60I - Examen TP: %d tri. - solid shading - %d FPS.",
                     numOfTriangles, FPS);
        else if (mode == Phong)
            sprintf (FPSstr, "HAI60I - Examen TP: %d tri. - Phong shading - %d FPS.",
                     numOfTriangles, FPS);
        glutSetWindowTitle (FPSstr);
        lastTime = currentTime;

    }
    // Pour une animation continue, ca se passe ici! Dans la fonction qui se lance toutes les frames
    angle = currentTime / 1000.f;
    std::cout << "angle = " << angle << std::endl;
    updateAnimation();
    glutPostRedisplay ();
}

void printUsage () {
    cerr << endl
         << "--------------------------------------" << endl
         << "HAI60I - Examen TP" << endl
         << "--------------------------------------" << endl
         << "USAGE: ./Main <file>.off" << endl
         << "--------------------------------------" << endl
         << "Keyboard commands" << endl
         << "--------------------------------------" << endl
         << " ?: Print help" << endl
         << " w: Toggle wireframe Mode" << endl
         << " f: Toggle full screen mode" << endl
         << " A/a: Increase/Decrease ambient reflection" << endl
         << " D/d: Increase/Decrease diffuse reflection" << endl
         << " S/s: Increase/Decrease specular reflection" << endl
         << " +/-: Increase/Decrease shininess" << endl
         << " R/r : Augmente/Diminue l’angle de rotation" << endl
         << " T/t : Augmente/Diminue l’offset du maillage pour la translation" << endl
         << " <drag>+<left button>: rotate model" << endl
         << " <drag>+<right button>: move model" << endl
         << " <drag>+<middle button>: zoom" << endl
         << " q, <esc>: Quit" << endl << endl
         << "--------------------------------------" << endl;
}



void key (unsigned char keyPressed, int x, int y) {
    switch (keyPressed) {
    case 'f':
        if (fullScreen == true) {
            glutReshapeWindow (SCREENWIDTH, SCREENHEIGHT);
            fullScreen = false;
        } else {
            glutFullScreen ();
            fullScreen = true;
        }
        break;
    case 'q':
    case 27:
        clear ();
        exit (0);
        break;
    case 'w':
        if( mode == Wire ){
            glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
            phongShader->bind ();
            mode = Phong;
        } else {
            glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
            phongShader->bind ();
            mode = Wire;
        }
        break;

    case 'D':
        diffuseRef += 0.1f;
        phongShader->setDiffuseRef (diffuseRef);
        break;
    case 'd':
        if (diffuseRef > 0.1f) {
            diffuseRef -= 0.1f;
            phongShader->setDiffuseRef (diffuseRef);
        }
        break;
    case 'S':
        specularRef += 0.1f;
        phongShader->setSpecularRef (specularRef);
        break;
    case 's':
        if (specularRef > 0.1f) {
            specularRef -= 0.1f;
            phongShader->setSpecularRef (specularRef);
        }
        break;
    case 'A':
        ambientRef += 0.1f;
        phongShader->setAmbientRef (ambientRef);
        break;
    case 'a':
        if (ambientRef > 0.1f) {
            ambientRef -= 0.1f;
            phongShader->setAmbientRef (ambientRef);
        }
        break;
    case '+':
        shininess += 1.0f;
        phongShader->setShininess (shininess);
        break;
    case '-':
        if (shininess > 1.0f) {
            shininess -= 1.0f;
            phongShader->setShininess (shininess);
        }
        break;
    case 'N':
        levels ++;
        phongShader->setLevels(levels);
        break;
    case 'n':
        if (levels > 1) {
            levels --;
            phongShader->setLevels(levels);
        }
        break;

    case '1':
        setExercice(1);
        break;
    case '2':
        setExercice(2);
        break;
    case '3':
        setExercice(3);
        break;
    case '4':
        setExercice(4);
        break;
    case 'R':
        angle += 0.1f;
        if( angle >= (float)M_PI*2.f ) angle = 0.;
        updateAnimation();
        break;
    case 'r':
        angle -= 0.1f;
        if( angle <= 0.f ) angle = (float)M_PI*2.f;
        updateAnimation();
        break;
    case 'T':
        offset += 0.01f;
        updateAnimation();
        break;
    case 't':
        offset -= 0.01f;
        updateAnimation();
        break;


    case 'Y':
        buildModel(current_mesh, ++numberOfVerticesOnCircles);
        break;
    case 'y':
        numberOfVerticesOnCircles = std::max(3, numberOfVerticesOnCircles - 1);
        buildModel(current_mesh, numberOfVerticesOnCircles);
        break;

    case '?':
    default:
        printUsage ();
        break;
    }
    setShaderValues ();
    idle ();
}

void mouse (int button, int state, int x, int y) {
    if (state == GLUT_UP) {
        mouseMovePressed = false;
        mouseRotatePressed = false;
        mouseZoomPressed = false;
    } else {
        if (button == GLUT_LEFT_BUTTON) {
            camera.beginRotate (x, y);
            mouseMovePressed = false;
            mouseRotatePressed = true;
            mouseZoomPressed = false;
        } else if (button == GLUT_RIGHT_BUTTON) {
            lastX = x;
            lastY = y;
            mouseMovePressed = true;
            mouseRotatePressed = false;
            mouseZoomPressed = false;
        } else if (button == GLUT_MIDDLE_BUTTON) {
            if (mouseZoomPressed == false) {
                lastZoom = y;
                mouseMovePressed = false;
                mouseRotatePressed = false;
                mouseZoomPressed = true;
            }
        }
    }
    idle ();
}

void motion (int x, int y) {
    if (mouseRotatePressed == true)
        camera.rotate (x, y);
    else if (mouseMovePressed == true) {
        camera.move ((x-lastX)/static_cast<float>(SCREENWIDTH),
                     (lastY-y)/static_cast<float>(SCREENHEIGHT),
                     0.0);
        lastX = x;
        lastY = y;
    }
    else if (mouseZoomPressed == true) {
        camera.zoom (float (y-lastZoom)/SCREENHEIGHT);
        lastZoom = y;
    }
}

void usage () {
    printUsage ();
    exit (EXIT_FAILURE);
}



int main (int argc, char ** argv) {
    glutInit (&argc, argv);
    glutInitDisplayMode (GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize (SCREENWIDTH, SCREENHEIGHT);
    window = glutCreateWindow ( "HAI605I - Revision Examen");


    init ();

    glutIdleFunc (idle);
    glutDisplayFunc (display);
    glutKeyboardFunc (key);
    glutReshapeFunc (reshape);
    glutMotionFunc (motion);
    glutMouseFunc (mouse);

    key ('?', 0, 0);

    glDepthFunc (GL_LESS);
    glEnable (GL_DEPTH_TEST);

    phongShader->bind ();
    glutMainLoop ();
    return EXIT_SUCCESS;
}

