// TinkerToy.cpp : Defines the entry point for the console application.
//
#include <iostream>
using namespace std;
#include "Gravity.h"
#include "WallLeft.h"
#include "WallRight.h"
#include <algorithm>
#include <list>
#include <Windows.h>

#include "Particle.h"
#include "SpringForce.h"
#include "AngularForce.h"
#include "RodConstraint.h"
#include "DragForce.h"
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

static void init_systemSpring(void)
{
	float distance;
	float strength;
	float damping;
	cout << "give distance between 2 particles please" << endl;
	cin >> distance;
	cout << "give spring strenght please" << endl;
	cin >> strength;
	cout << "give a spring damping please" << endl;
	cin >> damping;
	level_start_time = timeGetTime();

	int particleID = 0;
	pVector.push_back(new Particle(Vec2f(0.0, 0.7), 1.0f, particleID++, 0));
	pVector.push_back(new Particle(Vec2f(0.0, 0.1), 1.0f, particleID++, 0));

	int i, size = pVector.size();

	for (i = 0; i<size; i++)
	{
		mouses.push_back(new MouseForce(pVector[i], pVector[i]->m_Velocity, 0.5, 0.5));
	}

	for (i = 0; i<size; i++)
	{
		forces.push_back(new Gravity(pVector[i], Vec2f(0.0, -0.0981)));
	}

	constraints.push_back(new CircularWireConstraint(pVector[0], Vec2f(0.0,0.8), 0.1));
	forces.push_back(new SpringForce(pVector[0], pVector[1], distance, strength, damping, 0));

	for (i = 0; i<size; i++)
	{
		forces.push_back(new DragForce(pVector[i], 0.99));
	}
}

static void init_systemRod(void)
{
	float distance;
	cout << "give distance between 2 particles please" << endl;
	cin >> distance;
	level_start_time = timeGetTime();

	int particleID = 0;
	pVector.push_back(new Particle(Vec2f(0.0, 0.7), 1.0f, particleID++, 0));
	pVector.push_back(new Particle(Vec2f(0.0, 0.7-distance), 1.0f, particleID++, 0));

	int i, size = pVector.size();

	for (i = 0; i<size; i++)
	{
		mouses.push_back(new MouseForce(pVector[i], pVector[i]->m_Velocity, 0.5, 0.5));
	}

	for (i = 0; i<size; i++)
	{
		forces.push_back(new Gravity(pVector[i], Vec2f(0.0, -0.0981)));
	}

	constraints.push_back(new CircularWireConstraint(pVector[0], Vec2f(0.0, 0.8), 0.1));
	constraints.push_back(new RodConstraint(pVector[0], pVector[1], distance));

	for (i = 0; i<size; i++)
	{
		forces.push_back(new DragForce(pVector[i], 0.99));
	}
}

static void init_systemLine(void)
{
	float height;
	int wall;
	cout << "give the height of the line please" << endl;
	cin >> height;
	cout << "should there be a wall? (0 no, 1 yes on the left, 2 yes on both sides)" << endl;
	cin >> wall;
	level_start_time = timeGetTime();

	int particleID = 0;
	pVector.push_back(new Particle(Vec2f(0.0, height), 1.0f, particleID++, 0));
	pVector.push_back(new Particle(Vec2f(0.0, -1.1), 1.0f, particleID++, 0));

	int i, size = pVector.size();

	for (i = 0; i<size; i++)
	{
		mouses.push_back(new MouseForce(pVector[i], pVector[i]->m_Velocity, 0.5, 0.5));
	}

	for (i = 0; i<size; i++)
	{
		forces.push_back(new Gravity(pVector[i], Vec2f(0.0, -0.0981)));
	}

	constraints.push_back(new LineWireConstraint(pVector[0], height));
	forces.push_back(new SpringForce(pVector[0], pVector[1], 0.5, 3, 1, 0));

	if (wall == 0)
	{

	}
	else if (wall == 1)
	{
		float xPos1;
		cout << "please give the x position of the left wall";
		cin >> xPos1;
		forces.push_back(new WallLeft(pVector, xPos1, dt));
	}
	else if (wall == 2)
	{
		float xPos1;
		float xPos2;
		cout << "please give the x position of the left wall";
		cin >> xPos1;
		cout << "please give the x position of the right wall";
		cin >> xPos2;
		forces.push_back(new WallLeft(pVector, xPos1, dt));
		forces.push_back(new WallRight(pVector, xPos2, dt)); 
	}

	for (i = 0; i<size; i++)
	{
		forces.push_back(new DragForce(pVector[i], 0.99));
	}
}

static void init_systemCircle(void)
{
	int angle;
	level_start_time = timeGetTime();

	int particleID = 0;
	pVector.push_back(new Particle(Vec2f(0.0, 0.5), 1.0f, particleID++, 0));

	int i, size = pVector.size();

	for (i = 0; i<size; i++)
	{
		mouses.push_back(new MouseForce(pVector[i], pVector[i]->m_Velocity, 0.5, 0.5));
	}

	for (i = 0; i<size; i++)
	{
		forces.push_back(new Gravity(pVector[i], Vec2f(0.0, -0.0981)));
	}

	constraints.push_back(new CircularWireConstraint(pVector[0], Vec2f(0.0, 0.0), 0.5));

	for (i = 0; i<size; i++)
	{
		forces.push_back(new DragForce(pVector[i], 0.99));
	}

}

static void init_systemAngle(void)
{
	int angle;
	cout << "give angle please" << endl;
	cin >> angle;
	level_start_time = timeGetTime();

	int particleID = 0;
	pVector.push_back(new Particle(Vec2f(-0.2, 0.0), 1.0f, particleID++, 0));
	pVector.push_back(new Particle(Vec2f( 0.0, 0.4), 1.0f, particleID++, 0));
	pVector.push_back(new Particle(Vec2f(0.2 , 0.0), 1.0f, particleID++, 0));

	int i, size = pVector.size();

	for (i = 0; i<size; i++)
	{
		mouses.push_back(new MouseForce(pVector[i], pVector[i]->m_Velocity, 0.5, 0.5));
	}

	for (i = 0; i<size; i++)
	{
		forces.push_back(new Gravity(pVector[i], Vec2f(0.0, -0.0981)));
	}
	//forces.push_back(new Gravity(pVector[2], Vec2f(0.0, -0.981))); 

	forces.push_back(new SpringForce(pVector[0], pVector[1], 0.5, 2.0, 2.0, 0));
	forces.push_back(new SpringForce(pVector[1], pVector[2], 0.5, 2.0, 2.0, 0));
	constraints.push_back(new CircularWireConstraint(pVector[1], Vec2f(0.0, 0.5), 0.1));
	forces.push_back(new AngularForce(pVector[0], pVector[1], pVector[2], angle, 0.1, 0.5));

	for (i = 0; i<size; i++)
	{
		forces.push_back(new DragForce(pVector[i], 0.99 ));
	}
}


static void init_systemCloth(void)
{
	const double dist = 0.2;

	level_start_time = timeGetTime();

	int clothWidth;
	int clothHeight;
	int ks;
	int kd;
	cout << "please give a particle with and a height for the cloth" << endl;
	cout << "please give width: ";
	cin >> clothWidth;
	cout << "please give height: ";
	cin >> clothHeight;
	cout << "please give spring strength: ";
	cin >> ks;
	cout << "please give spring damping: ";
	cin >> kd;
	float distance = 0.0;
	float heigthOff = 0.0;
	float clothDist = 0.2f;
	int particleID = 0;

	for (int i = 0; i < clothWidth*clothHeight; i++)
	{
		if (i % clothWidth == 0)
		{
			heigthOff += 0.2;
			distance = 0.0;
		}

		pVector.push_back(new Particle(Vec2f(-0.3 + distance, 0.9 - heigthOff), 1.0f, particleID++, 0));
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


	//connect particles to the one next to him vertically
	for (int i = 0; i < (clothWidth*clothHeight)-clothWidth; i++)
	{
		forces.push_back(new SpringForce(pVector[i], pVector[i+clothWidth], clothDist, ks, kd, 0));
	}

	//connect particles with the one below him
	for (int i = 0; i < clothHeight; i++)
	{
		for (int j = 0; j < clothWidth - 1; j++)
		{
			forces.push_back(new SpringForce(pVector[j + (clothWidth * i)], pVector[j + (clothWidth * i) + 1], clothDist, ks, kd, 0));
		}
	}

	//connects vertically particles with the particle after it's next one
	for (int i = 0; i < clothHeight; i++)
	{
		for (int j = 0; j < clothWidth - 2; j++)
		{ 
			forces.push_back(new SpringForce(pVector[j + (i*clothWidth)], pVector[j + 2 + (i*clothWidth)], clothDist*2, ks, kd, 0));
		}
	}

	//connect horizontally particles with the particle after it's next one
	for (int i = 0; i < clothHeight-2; i++)
	{
		for (int j = 0; j < clothWidth; j++)
		{
			forces.push_back(new SpringForce(pVector[j + (clothWidth*i)], pVector[j + (clothWidth*i)+(clothWidth * 2)], clothDist * 2, ks, kd, 0));
		}
	}

	//cross connection left to right
	float clothDistCross = sqrt(clothDist*clothDist + clothDist*clothDist);
	for (int i = 0; i < clothWidth-1; i++)
	{
		for (int j = 0; j < clothHeight-1 ; j++)
		{
			forces.push_back(new SpringForce(pVector[i + (j*clothWidth)], pVector[i+1 + ((j+1)*clothWidth)], clothDistCross, ks, kd, 0));
		}
	}

	//cross connection right to left
	for (int i = 0; i < clothWidth - 1; i++)
	{
		for (int j = 0; j < clothHeight - 1; j++)
		{
			forces.push_back(new SpringForce(pVector[i+1 + (j*clothWidth)], pVector[i+((j+1)*clothWidth)], clothDistCross, ks, kd, 0));
		}
	}

	//cloth constraints.

	for (int i = 0; i < clothWidth; i++)
	{
		constraints.push_back(new LineWireConstraint(pVector[i], 0.7));
	}

	forces.push_back(new WallLeft(pVector, -1, dt));
	forces.push_back(new WallRight(pVector, 1, dt));

	for (i = 0; i<size; i++)
	{
		forces.push_back(new DragForce(pVector[i], 0.99));
	}
}

static void init_systemFlag(void)
{
	const double dist = 0.2;

	level_start_time = timeGetTime();

	int clothWidth = 12;
	int clothHeight = 7;
	int ks = 2;
	int kd = 1;
	cout << "Welcome, please enjoy the flag" << endl;
	float distance = 0.0;
	float heigthOff = 0.0;
	float clothDist = 0.16f;
	int particleID = 0;
	int colour = 0;

	for (int i = 0; i < clothWidth*clothHeight; i++)
	{
		if (i % clothWidth == 0)
		{
			heigthOff += clothDist;
			distance = 0.0;
		}

		if (i < clothWidth * 2)
		{
			colour = 0;
		}
		else if (i < clothWidth * 4)
		{
			colour = 1;
		}
		else
		{
			colour = 2;
		}
		pVector.push_back(new Particle(Vec2f(-0.9 + distance, 0.9 - heigthOff), 1.0f, particleID++, colour));
		distance += clothDist;
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
		//forces.push_back(new Gravity(pVector[i], Vec2f(0.0, -0.00981)));
	}

	//connect particles to the one below him
	for (int i = 0; i < (clothWidth*clothHeight) - clothWidth; i++)
	{
		if (i < clothWidth * 2)
		{
			colour = 0;
		}
		else if ( i < clothWidth * 4)
		{
			colour = 1;
		}
		else
		{
			colour = 2;
		}
		forces.push_back(new SpringForce(pVector[i], pVector[i + clothWidth], clothDist, ks, kd, colour));
	}

	//connect particles with the one next to him 
	for (int i = 0; i < clothHeight; i++)
	{
		for (int j = 0; j < clothWidth - 1; j++)
		{
			if (i == 0 || i == 1)
			{
				colour = 0;
			}
			if (i == 2 || i == 3)
			{
				colour = 1;
			}
			if (i == 4 || i == 5)
			{
				colour = 2;
			}
			forces.push_back(new SpringForce(pVector[j + (clothWidth * i)], pVector[j + (clothWidth * i) + 1], clothDist, ks, kd, colour));
		}
	}

	//connects vertically particles with the particle after it's next one
	for (int i = 0; i < clothHeight; i++)
	{
		for (int j = 0; j < clothWidth - 2; j++)
		{
			if (i == 0 ||i == 1)
			{
				colour = 0;
			}
			else if (i == 2 || i == 3 )
			{
				colour = 1;
			}
			else if (i == 4 || i == 5 )
			{
				colour = 2;
			}
			forces.push_back(new SpringForce(pVector[j + (i*clothWidth)], pVector[j + 2 + (i*clothWidth)], clothDist * 2, ks, kd, colour));
		}
	}

	//connect horizontally particles with the particle after it's next one
	for (int i = 0; i < clothHeight - 2; i++)
	{
		for (int j = 0; j < clothWidth; j++)
		{
			if (i == 0 || i == 1)
			{
				colour = 0;
			}
			else if (i == 2 || i == 3)
			{
				colour = 1;
			}
			else if (i == 4 || i == 5)
			{
				colour = 2;
			}
			forces.push_back(new SpringForce(pVector[j + (clothWidth*i)], pVector[j + (clothWidth*i) + (clothWidth * 2)], clothDist * 2, ks, kd, colour));
		}
	}

	//cross connection left to right
	float clothDistCross = sqrt(clothDist*clothDist + clothDist*clothDist);
	for (int i = 0; i < clothWidth - 1; i++)
	{
		for (int j = 0; j < clothHeight - 1; j++)
		{
			if (j == 0 || j == 1)
			{
				colour = 0;
			}
			else if ( j== 2 || j == 3 )
			{
				colour = 1;
			}
			else
			{
				colour = 2;
			}
			forces.push_back(new SpringForce(pVector[i + (j*clothWidth)], pVector[i + 1 + ((j + 1)*clothWidth)], clothDistCross, ks, kd, colour));
			forces.push_back(new SpringForce(pVector[i + 1 + (j*clothWidth)], pVector[i + ((j + 1)*clothWidth)], clothDistCross, ks, kd, colour));
		}
	}

	//cloth constraints.
	forces.push_back(new WallLeft(pVector, -1, dt));
	forces.push_back(new WallRight(pVector, 1, dt));

	//constraints.push_back(new CircularWireConstraint(pVector[0], Vec2f(-1, 0.7), 0.1));
	//constraints.push_back(new CircularWireConstraint(pVector[(clothWidth*clothHeight)-clothWidth], Vec2f(-1, -0.2), 0.1));

	for (i = 0; i<size; i++)
	{
		forces.push_back(new DragForce(pVector[i], 0.99));
	}
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
	while (!(((level_elapsed_time - level_start_time) % 17) == 0))
	{
		level_elapsed_time = timeGetTime();
		Sleep(1);
	}
	
	if (((level_elapsed_time - level_start_time) % 1000) <= 16)
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

	printf("\n\nWelcome to the demo  of Sander Kools and Giedo Mak\n" );
	printf(" \t please select what you want to view\n");
	printf(" \t give '1' as input for the spring force demo \n");
	printf(" \t give '2' as input for the angle force demo \n");
	printf(" \t give '3' as input for the circular wire constraint demo \n" );
	printf(" \t give '4' as input for the line wire constraint demo \n");
	printf(" \t give '5' as input for the rod constraint demo \n");
	printf(" \t give '6' as input for the cloth simulation demo \n");
	printf(" \t give '7' as input for the flag enjoyment \n");
	int input;
	cin >> input;
	if ( input == 1)
	{
		init_systemSpring();
	}
	else if ( input == 2)
	{
		init_systemAngle();
	}
	else if ( input == 3 )
	{
		init_systemCircle();
	}
	else if ( input == 4 )
	{
		init_systemLine();
	}
	else if ( input == 5 )
	{
		init_systemRod();
	}
	else if (input == 6)
	{
		init_systemCloth();
	}
	else if (input == 7)
	{
		init_systemFlag();
	}

	dump_frames = 0;
	frame_number = 0;

	win_x = 1024;
	win_y = 1024;
	open_glut_window ();

	dsim = 0;
	mouse_down[0] = true;
	remap_GUI();
	glutMainLoop ();

	exit ( 0 );
}
