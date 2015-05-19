// TinkerToy.cpp : Defines the entry point for the console application.
//
#include <iostream>
using namespace std;
#include "Gravity.h"
#include "Wall.h"
#include <algorithm>
#include <list>
#include <Windows.h>

#include "Particle.h"
#include "SpringForce.h"
#include "AngularForce.h"
#include "RodConstraint.h"
#include "MouseForce.h"
#include "CircularWireConstraint.h"
#include "PointConstraint.h"
#include "LineWireConstraint.h"
#include "imageio.h"
#include "Force.h"

#include <vector>
#include <stdlib.h>
#include <stdio.h>

#if defined(_WIN32)
#  include <glut.h>
#elif defined(__APPLE__)
#  include <GLUT/glut.h>
#endif
// #include <glut.h>
/* macros */

/* external definitions (from solver) */
extern void simulation_step(std::vector<Particle*> pVector, std::vector<Force*> forces, std::vector<Constraint*> constraints, float dt, int solver);

/* global variables */

static int N;
static float dt, d;
int dsim;
static int dump_frames;
static int frame_number;

// static Particle *pList;
static std::vector<Particle*> pVector;

Vec2f MousePos;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;

int particleSelected = -1;

std::list<RodConstraint*> rods;
std::list<CircularWireConstraint*> circles;
std::list<LineWireConstraint*> lines;

std::vector<MouseForce*> mouses;
std::vector<Force*> forces;
std::vector<Constraint*> constraints;

long long level_elapsed_time = 0;
long long level_start_time = 0;

int solverMethod = 1;

/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data ( void )
{
	pVector.clear();
}

static void clear_data ( void )
{
	int i, size = pVector.size();

	for(i=0; i<size; i++){
		pVector[i]->reset();
	}
}

static void init_system(void)
{
	const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);
	const Vec2f secondRow(0.0, -0.2);
	const Vec2f thirdRow(0.0, -0.4);
	const Vec2f fourthRow(0.0, -0.6);

	level_start_time = timeGetTime();

	// Create three particles, attach them to each other, then add a
	// circular wire constraint to the first.

	//pVector.push_back(new Particle(Vec2f(0.0, 0.0) + Vec2f(dist, 0.0)));
	//pVector.push_back(new Particle((center - offset), 1.0f, 0));
	//pVector.push_back(new Particle(Vec2f(0.0, 0.2), 1.0f, 1));
	//pVector.push_back(new Particle(Vec2f(0.0, -0.1), 1.0f, 2));

	//cloth particles
	/*
	float distance = 0.0;
	float heigthOff = 0.0;
	float clothDist = 0.2f;
	int clothWidth = 3;
	int clothHeight = 7;
	int particleID = 0;

	for (int i = 0; i < clothWidth*clothHeight; i++)
	{
		if (i % clothWidth == 0)
		{
			heigthOff += 0.2;
			distance = 0.0;
		}

		pVector.push_back(new Particle(Vec2f(-0.3 + distance, 0.7 - heigthOff), 1.0f, particleID++));
		distance += 0.2;
	}
	// You shoud replace these with a vector generalized forces and one of
	// constraints...

	int i, size = pVector.size();

	for (i = 0; i<size; i++)
	{
		mouses.push_back(new MouseForce(pVector[i], pVector[i]->m_Velocity, 0.5, 0.5));
	}

	for (i = 0; i<size; i++)
	{
		forces.push_back(new Gravity(pVector[i], Vec2f(0.0, -0.0981)));
	}

	//cloth forces
	int ks = 5;
	int kd = 3;

	//forces.push_back(new SpringForce(pVector[1], pVector[2], 0.05, 3, 1));
	for (int i = 0; i < (clothWidth*clothHeight)-clothWidth; i++)
	{
		//constraints.push_back(new RodConstraint(pVector[i], pVector[i + clothWidth], clothDist));
		forces.push_back(new SpringForce(pVector[i], pVector[i+clothWidth], clothDist, ks, kd));
	}
	for (int i = 0; i < clothHeight; i++)
	{
		for (int j = 0; j < clothWidth-1; j++)
		{
			//constraints.push_back(new RodConstraint(pVector[j + (clothWidth * i)], pVector[j + (clothWidth * i) + 1], clothDist));
			forces.push_back(new SpringForce(pVector[j + (clothWidth * i)], pVector[j + (clothWidth * i) + 1], clothDist, ks, kd));
		}
	}
	//cloth constraints.

	for (int i = 0; i < clothWidth; i++)
	{
		constraints.push_back(new LineWireConstraint(pVector[i], 0.5));
	}

	forces.push_back(new Wall(pVector, -0.5, dt));
	*/
	int particleID = 0;
	pVector.push_back(new Particle(Vec2f(-0.3, 0.5), 1.0f, particleID++));
	pVector.push_back(new Particle(Vec2f(0.2, 0.5), 1.0f, particleID++));
	pVector.push_back(new Particle(Vec2f(0.2, 0.0), 1.0f, particleID++));

	int i, size = pVector.size();

	for (i = 0; i<size; i++)
	{
		mouses.push_back(new MouseForce(pVector[i], pVector[i]->m_Velocity, 0.5, 0.5));
	}

	for (i = 0; i<size; i++)
	{
		//forces.push_back(new Gravity(pVector[i], Vec2f(0.0, -0.0981)));
	}
	//forces.push_back(new Gravity(pVector[2], Vec2f(0.0, -0.981))); 

	forces.push_back(new SpringForce(pVector[0], pVector[1], 0.5, 2.0, 2.0));
	forces.push_back(new SpringForce(pVector[1], pVector[2], 0.5, 2.0, 2.0));
	//constraints.push_back(new RodConstraint(pVector[1], pVector[2], 0.5));
	constraints.push_back(new LineWireConstraint(pVector[1], 0.5));
	//constraints.push_back(new LineWireConstraint(pVector[0], 0.5));
	//constraints.push_back(new CircularWireConstraint(pVector[1], Vec2f(0.0, 0.0), 0.3));
	//forces.push_back(new Wall(pVector, -0.6, dt));
	forces.push_back(new AngularForce(pVector[0], pVector[1], pVector[2], 90, 0.01, 0.5));
}

/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/


static void pre_display ( void )
{
	glViewport ( 0, 0, win_x, win_y );
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluOrtho2D ( -1.0, 1.0, -1.0, 1.0 );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
	// Write frames if necessary.
	//cout << "time: " << (level_elapsed_time - level_start_time) << endl;
	level_elapsed_time = timeGetTime();
	while(!(((level_elapsed_time - level_start_time) % 33) == 0))
	{
		level_elapsed_time = timeGetTime();
		Sleep(1);
	}
	
	if (((level_elapsed_time - level_start_time) % 1000) <= 33)
	{
		cout << "frames per second: " << frame_number << endl;
		frame_number = 0;
	}
	frame_number++;

	glutSwapBuffers ();
}

static void draw_particles ( void )
{
	int size = pVector.size();

	for(int i=0; i< size; i++)
	{
		pVector[i]->draw();
	}
}

static void draw_forces ( void )
{
	for_each(forces.begin(), forces.end(), [](Force* f)
	{
		f->draw();
	});

	for_each(constraints.begin(), constraints.end(), [](Constraint* c)
	{
		c->draw();
	});

	for_each(mouses.begin(), mouses.end(), [](MouseForce* m)
	{
		m->draw();
	});
}

/*
----------------------------------------------------------------------
relates mouse movements to tinker toy construction
----------------------------------------------------------------------
*/

static void get_mouse_pos()
{
	//screen is -1 to 1 in both x and y pos
	//mouse pos is from 0 to 64
	float x = 0;
	float y = 0;
	
	int i, j;
	i = (int)((mx / (float)win_x)*N);
	j = (int)(((win_y - my) / (float)win_y)*N);

	if (!mouse_down[0] && !mouse_down[2] && !mouse_release[0]
		&& !mouse_shiftclick[0] && !mouse_shiftclick[2]) return;

	if (mouse_down[0]) 
	{

		x = i - 32;
		x = (float)(x / 32);

		y = j - 32;
		y = (float)(y / 32);

		int i, size = pVector.size();

		for (i = 0; i<size; i++)
		{

			MousePos[0] = x;
			MousePos[1] = y;

			float xDis = pVector[i]->m_Position[0] - MousePos[0];
			float yDis = pVector[i]->m_Position[1] - MousePos[1];

			float distance = xDis*xDis + yDis*yDis;

			if (distance < 0.001)
			{
				particleSelected = i;
			}

			//particle is selected
			if (particleSelected == i)
			{
				mouses[i]->getMouse(MousePos);
				mouses[i]->setForce(true);
				mouses[i]->apply();
			}
			else
			{
				mouses[i]->getMouse(pVector[i]->m_Position);
				mouses[i]->setForce(false);
			}

		}
	}
	else
	{
		particleSelected = -1;
		int i, size = pVector.size();

		for (i = 0; i < size; i++)
		{
			mouses[i]->getMouse(pVector[i]->m_Position);
			mouses[i]->setForce(false);
		}
	}
}



static void remap_GUI()
{
	int ii, size = pVector.size();
	for(ii=0; ii<size; ii++)
	{
		pVector[ii]->m_Position[0] = pVector[ii]->m_ConstructPos[0];
		pVector[ii]->m_Position[1] = pVector[ii]->m_ConstructPos[1];
	}
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/

static void key_func ( unsigned char key, int x, int y )
{
	switch ( key )
	{
	case '1':
		solverMethod = 1;
		break;

	case '2':
		solverMethod = 2;
		break;

	case '3':
		solverMethod = 3;
		break;

	case '4':
		solverMethod = 4;
		break;

	case 'c':
	case 'C':
		clear_data ();
		break;

	case 'd':
	case 'D':
		dump_frames = !dump_frames;
		break;

	case 'q':
	case 'Q':
		free_data ();
		exit ( 0 );
		break;

	}
}

static void mouse_func ( int button, int state, int x, int y )
{
	omx = mx = x;
	omx = my = y;

	if(!mouse_down[0]){hmx=x; hmy=y;}
	if(mouse_down[button]) mouse_release[button] = state == GLUT_UP;
	if(mouse_down[button]) mouse_shiftclick[button] = glutGetModifiers()==GLUT_ACTIVE_SHIFT;
	mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func ( int x, int y )
{
	mx = x;
	my = y;
}

static void reshape_func ( int width, int height )
{
	glutSetWindow ( win_id );
	glutReshapeWindow ( width, height );

	win_x = width;
	win_y = height;
}

static void idle_func ( void )
{
	//euler = 1, midpoint = 2 and runge-kutta = 3
	simulation_step(pVector, forces, constraints, dt, solverMethod);
	get_mouse_pos();

	glutSetWindow ( win_id );
	glutPostRedisplay ();
}

static void display_func ( void )
{
	pre_display ();

	draw_forces();
	draw_particles();

	post_display ();
}


/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window ( void )
{
	glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );

	glutInitWindowPosition ( 0, 0 );
	glutInitWindowSize ( win_x, win_y );
	win_id = glutCreateWindow ( "Tinkertoys!" );

	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	pre_display ();

	glutKeyboardFunc ( key_func );
	glutMouseFunc ( mouse_func );
	glutMotionFunc ( motion_func );
	glutReshapeFunc ( reshape_func );
	glutIdleFunc ( idle_func );
	glutDisplayFunc ( display_func );
}


/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
	glutInit ( &argc, argv );

	if ( argc == 1 ) {
		N = 64;
		dt = 0.1f;
		d = 5.f;
		fprintf ( stderr, "Using defaults : N=%d dt=%g d=%g\n",
			N, dt, d );
	} else {
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf ( "\n\nHow to use this application:\n\n" );
	printf ( "\t Toggle construction/simulation display with the spacebar key\n" );
	printf ( "\t Dump frames by pressing the 'd' key\n" );
	printf ( "\t Quit by pressing the 'q' key\n" );

	dump_frames = 0;
	frame_number = 0;

	init_system();


	win_x = 512;
	win_y = 512;
	open_glut_window ();

	dsim = 0;
	mouse_down[0] = true;
	remap_GUI();
	glutMainLoop ();

	exit ( 0 );
}
