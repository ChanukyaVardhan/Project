/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 251 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * 
 */


#include "cs251_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
    #include <GLUT/glut.h>
#else
    #include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs251
{
    /********************************************//**
   * <b>This is the documentation block for the constructor</b> \n
   * <b> The pendulum that knocks the dominos off </b> <br>
   * - b2Body* b2 - the small vertical pillar to which the bob is connected \n
   * - shape - definition of shape of the pillar \n
   * - b2BodyDef bd - body definition of the pillar \n
   * - b2Body* b4 - bob of the pendulum \n
   * - b2PolygonShape shape - definition of shape of the bob \n
   * - b2BodyDef bd - body definition of the bob \n
   * - b2RevoluteJointDef jd - the joint definition that connects the bob and the pillar to form the pendulum
   * - b2Vec2 anchor - specifies the anchor point for the pendulum \n
   *
   * <b> Top horizontal shelf </b> <br>
   * (Upon which the Dominos lie)
   * - b2PolygonShape shape - definition of shape of the shelf \n
   * - b2BodyDef bd - body definition of the shelf \n
   *
   * <b> Dominos </b> <br>
   * (lie on the top horizontal shelf)
   * - b2PolygonShape shape - definition of shape of dominos \n
   * - b2FixtureDef fd - fixture definition of dominos \n
   * - b2BodyDef bd - body definition of each domino \n
   *
   * <b> Another Horizontal Shelf </b> <br>
   * (Upon which the train of spheres lie)
   * - b2PolygonShape shape - definition of shape of the shelf \n
   * - b2BodyDef bd - body definition of the shelf \n
   *
   * <b> Train of spheres </b> <br>
   * (lie on the second horizontal shelf)
   * - b2Body* spherebody - the train of spheres on the shelf \n
   * - b2CircleShape circle - definition of shape of the spheres \n
   * - b2FixtureDef ballfd - fixture definition of the spheres, properties like density friction and restitution are defined \n
   * - b2BodyDef ballbd - body definition of each sphere \n
   *
   * <b> The pulley system </b> <br>
   * - b2BodyDef *bd - Body definition of the horizontal rods \n
   *
   * - Open Box \n
   * &nbsp;&nbsp;&nbsp;- b2Body* box1 - body object for the open box on the right of the  pulley system \n
   * &nbsp;&nbsp;&nbsp;- b2FixtureDef *fd1, *fd2, *fd3 - fixture definitions of the three rods constituting the open box \n
   *
   * - The bar \n
   * &nbsp;&nbsp;&nbsp;- b2Body* box2 - body object for the bar on right side of the pulley system \n
   * 
   * - Pulley joint \n
   * &nbsp;&nbsp;&nbsp;- b2PulleyJointDef* myjoint - Pulley joint connecting the open box on the left and bar on the right \n
   * &nbsp;&nbsp;&nbsp;- b2Vec2 worldAnchorOnBody1, worldAnchorOnBody2, worldAnchorOnBody3 - anchor points for the pulley joint \n
   *
   * <b> Revolving Horiontal platform </b> <br>
   * (Upon which the heavy sphere lies) \n
   * - b2Body* body - body object corresponding to horizontal platform \n 
   * - b2PolygonShape shape - definition of shape of the platform \n
   * - b2BodyDef bd - defintion of body of platform \n
   * - b2FixtureDef *fd - fixture definition of the platform, density and shape are set \n
   * - b2Body* body2 - body object corresponding to pivot for the platform \n
   * - b2PolygonShape shape2 - shape definition of the pivot \n
   * - b2BodyDef bd2 - definition of body of pivot \n
   * - b2RevoluteJointDef jointDef - revolute joint between the platform and pivot \n
   * 
   * <b> Heavy sphere on the platform </b> <br>
   * - b2Body* sbody - body object for the Heavy sphere
   * - b2CircleShape circle - shape definition of the sphere \n
   * - b2BodyDef ballbd - body definition of the sphere \n
   * - b2FixtureDef ballfd - fixture definition, friction density and restitution are set \n
   * 
   * <b>The see-saw system at the bottom</b> <br>
   * - The triangular wedge \n
   * &nbsp;&nbsp;&nbsp;- b2Body* sbody - body object for the triangular wedge \n
   * &nbsp;&nbsp;&nbsp;- b2PolygonShape poly - shape definition of the wedge \n
   * &nbsp;&nbsp;&nbsp;- b2Vec2 vertices[3] - array containing the vertices of the triangle \n
   * &nbsp;&nbsp;&nbsp;- b2BodyDef wedgebd - body definition of the wedge \n
   * &nbsp;&nbsp;&nbsp;- b2FixtureDef wedgefd - fixture definition, friction density and restitution are set \n
   *
   * - The plank on top of the wedge \n
   *&nbsp;&nbsp;&nbsp; - b2Body* body - body object for the plank \n
   *&nbsp;&nbsp;&nbsp; - b2PolygonShape shape - shape defintion of the plank \n
   *&nbsp;&nbsp;&nbsp; - b2BodyDef bd2 - body definition of the plank \n
   *&nbsp;&nbsp;&nbsp; - b2FixtureDef *fd2 - fixture definition, density and shape are set \n
   *&nbsp;&nbsp;&nbsp; - b2RevoluteJointDef jd - joint between plank and triangular wedge \n
   *&nbsp;&nbsp;&nbsp; - b2Vec2 anchor - anchor points between wedge and plank  \n
   *
   * - The light box on the right side of the see-saw \n
   *&nbsp;&nbsp;&nbsp; - b2Body* body3 - body object for the box \n
   *&nbsp;&nbsp;&nbsp; - b2PolygonShape shape2 - shape definition of the box \n
   *&nbsp;&nbsp;&nbsp; - b2BodyDef bd3 - body definition of the box \n
   *&nbsp;&nbsp;&nbsp; - b2FixtureDef *fd3 - fixture definition, density and shape are set \n
   *
   * <b> Rotating platform #2 </b> <br>
   * (Upon which the heavy sphere lies) \n
   * - b2Body* body - body object corresponding to horizontal platform \n 
   * - b2PolygonShape shape - definition of shape of the platform \n
   * - b2BodyDef bd - defintion of body of platform \n
   * - b2FixtureDef *fd - fixture definition of the platform, density and shape are set \n
   * - b2Body* body2 - body object corresponding to pivot for the platform \n
   * - b2PolygonShape shape2 - shape definition of the pivot \n
   * - b2BodyDef bd2 - definition of body of pivot \n
   * - b2RevoluteJointDef jointDef - revolute joint between the platform and pivot \n
   *
   * <b> Sphere beside the light box </b> <br>
   * - b2Body* sbody - body object for the Heavy sphere
   * - b2CircleShape circle - shape definition of the sphere \n
   * - b2BodyDef ballbd - body definition of the sphere \n
   * - b2FixtureDef ballfd - fixture definition, friction density and restitution are set \n
   *
   * <b> Dominos in the bottom </b> <br>
   * (lie on ground)
   * - b2PolygonShape shape - definition of shape of dominos \n
   * - b2FixtureDef fd - fixture definition of dominos \n
   * - b2BodyDef bd - body definition of each domino \n
   ***********************************************/         

    b2PrismaticJointDef launcherJoint;
    b2PrismaticJointDef launcherJoint2;
    b2Body* ballBody;
    b2Body* flipperleftbody;
    b2Body* flipperrightbody;
    //b2Body* flipperwheelrightbody;
    //b2Body* flipperwheelupbody;
    //b2Body* flipperwheelleftbody;
    //b2Body* flipperwheeldownbody;
    b2Body* flipperrotbody1;
    b2Body* flipperrotbody2;
    //b2RevoluteJointDef wheeljointDef1;
    b2BodyDef ballBodyDef;
    b2FixtureDef ballFixtureDef;
    dominos_t::dominos_t()
    {
         //Outer Box
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(0, 10); //slightly lower position

            b2Vec2 vs[5];

            vs[0].Set(20,-14);
            vs[1].Set(20, 34);
            vs[2].Set(-20,34);
            vs[3].Set(-20,-14);
            vs[4].Set(20,-14);
              
            b2ChainShape boxShape;
            boxShape.CreateChain(vs, 5);

            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
        }

         //Ball Director - Top
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(0, 10); //slightly lower position

            {
                int times = 58;//set the initial value
                b2Vec2 vs[times];

                float step = 1/(float)19;
                float t = 0;

                b2Vec2 v1;
                v1.Set(20,9);
                b2Vec2 v2;
                v2.Set(20,14);
                b2Vec2 v3;
                v3.Set(20,21);
                b2Vec2 v4;
                v4.Set(17,31);

                vs[0].Set(20,-14);

                for(int i = 1;i < 20;i++)
                {
                b2Vec2 pa = v1;
                pa *= ( (1-t)*(1-t)*(1-t) );
                b2Vec2 pb = v2;
                pb *= ( 3*t*(1-t)*(1-t) );
                b2Vec2 pc = v3;
                pc *= ( 3*t*t*(1-t) );
                b2Vec2 pd = v4;
                pd *= ( t*t*t );
                vs[i] = pa+pb+pc+pd;
                t+=step;
                }

                step = 1/(float)10;
                t = 0;

                v1.Set(17,31);
                v2.Set(16,33);
                v3.Set(15,34);
                v4.Set(14,34);

                for(int i = 20  ;i < 30;i++)
                {
                b2Vec2 pa = v1;
                pa *= ( (1-t)*(1-t)*(1-t) );
                b2Vec2 pb = v2;
                pb *= ( 3*t*(1-t)*(1-t) );
                b2Vec2 pc = v3;
                pc *= ( 3*t*t*(1-t) );
                b2Vec2 pd = v4;
                pd *= ( t*t*t );
                vs[i] = pa+pb+pc+pd;
                t+=step;
                }

                step = 1/(float)10;
                t = 0;

                v1.Set(-14,34);
                v2.Set(-20,33);
                v3.Set(-20,30);
                v4.Set(-20,28);

                for(int i = 30  ;i < 40;i++)
                {
                b2Vec2 pa = v1;
                pa *= ( (1-t)*(1-t)*(1-t) );
                b2Vec2 pb = v2;
                pb *= ( 3*t*(1-t)*(1-t) );
                b2Vec2 pc = v3;
                pc *= ( 3*t*t*(1-t) );
                b2Vec2 pd = v4;
                pd *= ( t*t*t );
                vs[i] = pa+pb+pc+pd;
                t+=step;
                }
                //this is the point
                //vs[40].Set(-10,-5);

                step = 1/(float)10;
                t = 0;

                v1.Set(-20,2);
                v2.Set(-18.5,1);
                v3.Set(-18.5,1);
                v4.Set(-20,0);

                for(int i = 40  ;i < 50;i++)
                {
                b2Vec2 pa = v1;
                pa *= ( (1-t)*(1-t)*(1-t) );
                b2Vec2 pb = v2;
                pb *= ( 3*t*(1-t)*(1-t) );
                b2Vec2 pc = v3;
                pc *= ( 3*t*t*(1-t) );
                b2Vec2 pd = v4;
                pd *= ( t*t*t );
                vs[i] = pa+pb+pc+pd;
                t+=step;
                }
                vs[50].Set(-20,0);
                vs[51].Set(-20,-5);
                vs[52].Set(-6.5,-10);
                vs[53].Set(-6.25,-10.25);
                vs[54].Set(-6.5,-10.5);
                vs[55].Set(-20,-6);
                vs[56].Set(-6.5,-10.5);
                vs[57].Set(-6.5,-14);

                b2ChainShape boxShape;
                boxShape.CreateChain(vs, times);

                b2FixtureDef boxFixtureDef;
                boxFixtureDef.shape = &boxShape;
                boxFixtureDef.density = 1;
                boxFixtureDef.restitution = 0.1f;

                b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
                staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

        //Bottom Support
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(0, 10); //slightly lower position
            {
                b2Vec2 vs[5];

                vs[0].Set(14,-14);
                vs[1].Set(14,-9.5);
                vs[2].Set(4.5,-12);
                vs[3].Set(4.5,-14);
                
                b2ChainShape boxShape;
                boxShape.CreateChain(vs, 4);

                b2FixtureDef boxFixtureDef;
                boxFixtureDef.shape = &boxShape;
                boxFixtureDef.density = 1;

                b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
                staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

         //Ball Director - Left
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(0, 10); //slightly lower position
            {
                int times = 20;//set the initial value
                b2Vec2 vs[times];

                float step = 1/(float)19;
                float t = 0;

                b2Vec2 v1;
                v1.Set(17,9);
                b2Vec2 v2;
                v2.Set(17,14);
                b2Vec2 v3;
                v3.Set(17,21);
                b2Vec2 v4;
                v4.Set(14,31);

                vs[0].Set(17,-14);

                for(int i = 1;i < 20;i++)
                {
                b2Vec2 pa = v1;
                pa *= ( (1-t)*(1-t)*(1-t) );
                b2Vec2 pb = v2;
                pb *= ( 3*t*(1-t)*(1-t) );
                b2Vec2 pc = v3;
                pc *= ( 3*t*t*(1-t) );
                b2Vec2 pd = v4;
                pd *= ( t*t*t );
                vs[i] = pa+pb+pc+pd;
                t+=step;
                }

                b2ChainShape boxShape;
                boxShape.CreateChain(vs, times);

                b2FixtureDef boxFixtureDef;
                boxFixtureDef.shape = &boxShape;
                boxFixtureDef.density = 1;

                b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
                staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

        //Flipper bat supporter - Right
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(0, 10); //slightly lower position

            {
                b2Vec2 vs[8];

                vs[0].Set(14,-7.5);
                vs[1].Set(14,-0.5);
                vs[2].Set(13.5,-0.5);
                vs[3].Set(13.5,-6.5);
                vs[4].Set(4.5,-10);
                vs[5].Set(4,-10.25);
                vs[6].Set(4.5,-10.5);
                vs[7].Set(14,-7.5);
                
                b2ChainShape boxShape;
                boxShape.CreateChain(vs, 8);

                b2FixtureDef boxFixtureDef;
                boxFixtureDef.shape = &boxShape;
                boxFixtureDef.density = 1;

                b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
                staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

        //Left Flipper Bat
        {
            b2PolygonShape shape;
            shape.SetAsBox(1.5f, 0.2f);
        
            b2BodyDef flipper;
            flipper.position.Set(-6.0f, -2.25f);
            flipper.type = b2_dynamicBody;
            flipper.allowSleep = false;
            //b2Body*
            flipperleftbody = m_world->CreateBody(&flipper);
            b2FixtureDef *fd = new b2FixtureDef;
            fd->density = 1.f;
            fd->shape = new b2PolygonShape;
            fd->shape = &shape;
            flipperleftbody->CreateFixture(fd);

            b2PolygonShape shape2;
            shape2.SetAsBox(0.2f, 2.0f);
            b2BodyDef bd2;
            bd2.position.Set(-6.0f, -0.25f);
            bd2.allowSleep = false;
            b2Body* body2 = m_world->CreateBody(&bd2);

            b2RevoluteJointDef jointDef;
            jointDef.bodyA = flipperleftbody;
            jointDef.bodyB = body2;
            jointDef.localAnchorA.Set(-1.5,0);
            jointDef.localAnchorB.Set(0,0);
            jointDef.collideConnected = false;
            jointDef.enableLimit = true;
            jointDef.lowerAngle = -35 * 0.0174532925;
            jointDef.upperAngle =  14.534 * 0.0174532925;
            m_world->CreateJoint(&jointDef);
        }

        //Right Flipper Bat
        {
            b2PolygonShape shape;
            shape.SetAsBox(1.5f, 0.2f);
        
            b2BodyDef flipper;
            flipper.position.Set(3.75f, -2.25f);
            flipper.type = b2_dynamicBody;
            flipper.allowSleep = false;
            //b2Body*
            flipperrightbody = m_world->CreateBody(&flipper);
            b2FixtureDef *fd = new b2FixtureDef;
            fd->density = 1.f;
            fd->shape = new b2PolygonShape;
            fd->shape = &shape;
            flipperrightbody->CreateFixture(fd);

            b2PolygonShape shape2;
            shape2.SetAsBox(0.2f, 2.0f);
            b2BodyDef bd2;
            bd2.position.Set(3.75f, -0.25f);
            bd2.allowSleep = false;
            b2Body* body2 = m_world->CreateBody(&bd2);

            b2RevoluteJointDef jointDef;
            jointDef.bodyA = flipperrightbody;
            jointDef.bodyB = body2;
            jointDef.localAnchorA.Set(1.5,0);
            jointDef.localAnchorB.Set(0,0);
            jointDef.collideConnected = false;
            jointDef.enableLimit = true;
            jointDef.lowerAngle = -14.534 * 0.0174532925;
            jointDef.upperAngle =  35 * 0.0174532925;
            m_world->CreateJoint(&jointDef);
        }

        //Left side rotaters wall
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(0, 10); //slightly lower position
            {
                int times = 17;//set the initial value
                b2Vec2 vs[times];

                float step = 1/(float)5;
                float t = 0;

                b2Vec2 v1;
                v1.Set(-17,10);
                b2Vec2 v2;
                v2.Set(-16,9);
                b2Vec2 v3;
                v3.Set(-15,8);
                b2Vec2 v4;
                v4.Set(-14,8);

                //vs[0].Set(17,-14);

                for(int i = 0;i < 5;i++)
                {
                b2Vec2 pa = v1;
                pa *= ( (1-t)*(1-t)*(1-t) );
                b2Vec2 pb = v2;
                pb *= ( 3*t*(1-t)*(1-t) );
                b2Vec2 pc = v3;
                pc *= ( 3*t*t*(1-t) );
                b2Vec2 pd = v4;
                pd *= ( t*t*t );
                vs[i] = pa+pb+pc+pd;
                t+=step;
                }

                step = 1/(float)5;
                t = 0;

                v1.Set(-14,7.75);
                v2.Set(-13,7);
                v3.Set(-12,7);
                v4.Set(-11,6);

                for(int i = 5;i < 10;i++)
                {
                b2Vec2 pa = v1;
                pa *= ( (1-t)*(1-t)*(1-t) );
                b2Vec2 pb = v2;
                pb *= ( 3*t*(1-t)*(1-t) );
                b2Vec2 pc = v3;
                pc *= ( 3*t*t*(1-t) );
                b2Vec2 pd = v4;
                pd *= ( t*t*t );
                vs[i] = pa+pb+pc+pd;
                t+=step;
                }

                vs[10].Set(-11,6);
                //vs[11].Set(-11,-2);

                step = 1/(float)5;
                t = 0;

                v1.Set(-11,-2);
                v2.Set(-12,-3);
                v3.Set(-13,-3);
                v4.Set(-14,-4);

                for(int i = 11;i < 16;i++)
                {
                b2Vec2 pa = v1;
                pa *= ( (1-t)*(1-t)*(1-t) );
                b2Vec2 pb = v2;
                pb *= ( 3*t*(1-t)*(1-t) );
                b2Vec2 pc = v3;
                pc *= ( 3*t*t*(1-t) );
                b2Vec2 pd = v4;
                pd *= ( t*t*t );
                vs[i] = pa+pb+pc+pd;
                t+=step;
                }
                vs[16].Set(-14,-4);

                b2ChainShape boxShape;
                boxShape.CreateChain(vs, times);

                b2FixtureDef boxFixtureDef;
                boxFixtureDef.shape = &boxShape;
                boxFixtureDef.density = 1;

                b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
                staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

        /*// Circle 1 in left side near rotators
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(-18, 10); //slightly lower position
            {
            b2CircleShape circleShape1;
            circleShape1.m_p.Set(0, 0); //position, relative to body position
            circleShape1.m_radius = 1; //radius
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &circleShape1;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }

            {
            b2CircleShape circleShape2;
            circleShape2.m_p.Set(0, 0); //position, relative to body position
            circleShape2.m_radius = 0.4; //radius
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &circleShape2;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }*/

        // Circle 2 in left side near rotators
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(-14, 10); //slightly lower position
            {
            b2CircleShape circleShape1;
            circleShape1.m_p.Set(0, 0); //position, relative to body position
            circleShape1.m_radius = 1; //radius
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &circleShape1;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }

            {
            b2CircleShape circleShape2;
            circleShape2.m_p.Set(0, 0); //position, relative to body position
            circleShape2.m_radius = 0.4; //radius
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &circleShape2;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

        // Circle 3 in left side near rotators
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(-16, 7); //slightly lower position
            {
            b2CircleShape circleShape1;
            circleShape1.m_p.Set(0, 0); //position, relative to body position
            circleShape1.m_radius = 1; //radius
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &circleShape1;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }

            {
            b2CircleShape circleShape2;
            circleShape2.m_p.Set(0, 0); //position, relative to body position
            circleShape2.m_radius = 0.4; //radius
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &circleShape2;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

        // Rotator 1
        {
            b2PolygonShape shape;
            shape.SetAsBox(1.3f, 0.1f);
          
            b2BodyDef bd;
            bd.position.Set(-17.0f, 15.0f);
            bd.type = b2_kinematicBody;
            b2Body* body = m_world->CreateBody(&bd);
            b2FixtureDef *fd = new b2FixtureDef;
            fd->density = 1.f;
            fd->shape = new b2PolygonShape;
            fd->shape = &shape;
            body->CreateFixture(fd);
            body->SetAngularVelocity( 5 );


            b2PolygonShape shape2;
            shape2.SetAsBox(0.1f, 1.3f);
            b2BodyDef bd2;
            bd2.position.Set(-17.0f, 15.0f);
            bd2.type = b2_kinematicBody;
            b2Body* body2 = m_world->CreateBody(&bd2);  //here we made a bd2 mistake
            b2FixtureDef *fd2 = new b2FixtureDef;
            fd2->density = 1.f;
            fd2->shape = new b2PolygonShape;
            fd2->shape = &shape2;
            body2->CreateFixture(fd2);
            body2->SetAngularVelocity( 5 );

            b2RevoluteJointDef jointDef;
            jointDef.bodyA = body;
            jointDef.bodyB = body2;
            jointDef.localAnchorA.Set(0,0);
            jointDef.localAnchorB.Set(0,0);
            jointDef.collideConnected = false;
            m_world->CreateJoint(&jointDef);
        }

        // Rotator 2
        {
            b2PolygonShape shape;
            shape.SetAsBox(1.3f, 0.1f);
          
            b2BodyDef bd;
            bd.position.Set(-14.0f, 14.0f);
            bd.type = b2_kinematicBody;
            b2Body* body = m_world->CreateBody(&bd);
            b2FixtureDef *fd = new b2FixtureDef;
            fd->density = 1.f;
            fd->shape = new b2PolygonShape;
            fd->shape = &shape;
            body->CreateFixture(fd);
            body->SetAngularVelocity( -5 );


            b2PolygonShape shape2;
            shape2.SetAsBox(0.1f, 1.3f);
            b2BodyDef bd2;
            bd2.position.Set(-14.0f, 14.0f);
            bd2.type = b2_kinematicBody;
            b2Body* body2 = m_world->CreateBody(&bd2);  //here we made a bd2 mistake
            b2FixtureDef *fd2 = new b2FixtureDef;
            fd2->density = 1.f;
            fd2->shape = new b2PolygonShape;
            fd2->shape = &shape2;
            body2->CreateFixture(fd2);
            body2->SetAngularVelocity( -5 );

            b2RevoluteJointDef jointDef;
            jointDef.bodyA = body;
            jointDef.bodyB = body2;
            jointDef.localAnchorA.Set(0,0);
            jointDef.localAnchorB.Set(0,0);
            jointDef.collideConnected = false;
            m_world->CreateJoint(&jointDef);
        }

        // Top red part without Hole
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(-10, 32); //slightly lower position

            b2Vec2 vertices[5];
            vertices[0].Set(-1,  5);
            vertices[1].Set(-1,  0);
            vertices[2].Set( 2,  2);
            vertices[3].Set( 2,  5);
            vertices[4].Set( 0,  6);
  

            b2PolygonShape boxShape;
            boxShape.Set(vertices,5);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
        }

        // Top Bumper Circle 1
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(-2 , 30); //slightly lower position
            {
            b2CircleShape circleShape1;
            circleShape1.m_p.Set(0, 0); //position, relative to body position
            circleShape1.m_radius = 2; //radius
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &circleShape1;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }

            {
            b2CircleShape circleShape2;
            circleShape2.m_p.Set(0, 0); //position, relative to body position
            circleShape2.m_radius = 0.8; //radius
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &circleShape2;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

        // Top Bumper Circle 2
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(5 , 31); //slightly lower position
            {
            b2CircleShape circleShape1;
            circleShape1.m_p.Set(0, 0); //position, relative to body position
            circleShape1.m_radius = 2; //radius
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &circleShape1;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }

            {
            b2CircleShape circleShape2;
            circleShape2.m_p.Set(0, 0); //position, relative to body position
            circleShape2.m_radius = 0.8; //radius
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &circleShape2;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

        // Top Bumper Circle 3
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(1.5 , 25); //slightly lower position
            {
            b2CircleShape circleShape1;
            circleShape1.m_p.Set(0, 0); //position, relative to body position
            circleShape1.m_radius = 2; //radius
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &circleShape1;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }

            {
            b2CircleShape circleShape2;
            circleShape2.m_p.Set(0, 0); //position, relative to body position
            circleShape2.m_radius = 0.8; //radius
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &circleShape2;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

        // Top Bumper to the left
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(-15 , 38); //slightly lower position
            {
            b2CircleShape circleShape1;
            circleShape1.m_p.Set(0, 0); //position, relative to body position
            circleShape1.m_radius = 2; //radius
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &circleShape1;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }

            {
            b2CircleShape circleShape2;
            circleShape2.m_p.Set(0, 0); //position, relative to body position
            circleShape2.m_radius = 0.8; //radius
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &circleShape2;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

        // Red Part obstacles on right side up
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(1.5, 10); //slightly lower position
            {
                int times = 22;//set the initial value
                b2Vec2 vs[times];

                float step = 1/(float)10;
                float t = 0;

                b2Vec2 v1;
                v1.Set(11,25);
                b2Vec2 v2;
                v2.Set(14,20);
                b2Vec2 v3;
                v3.Set(14,18);
                b2Vec2 v4;
                v4.Set(11,15);

                //vs[0].Set(17,-14);

                for(int i = 0;i < 10;i++)
                {
                b2Vec2 pa = v1;
                pa *= ( (1-t)*(1-t)*(1-t) );
                b2Vec2 pb = v2;
                pb *= ( 3*t*(1-t)*(1-t) );
                b2Vec2 pc = v3;
                pc *= ( 3*t*t*(1-t) );
                b2Vec2 pd = v4;
                pd *= ( t*t*t );
                vs[i] = pa+pb+pc+pd;
                t+=step;
                }

                vs[10].Set(11,15);
                //vs[11].Set(10,16);

                step = 1/(float)10;
                t = 0;

                v1.Set(10,16);
                v2.Set(13,20);
                v3.Set(12,20);
                v4.Set(11,25);

                for(int i = 11;i < 21;i++)
                {
                b2Vec2 pa = v1;
                pa *= ( (1-t)*(1-t)*(1-t) );
                b2Vec2 pb = v2;
                pb *= ( 3*t*(1-t)*(1-t) );
                b2Vec2 pc = v3;
                pc *= ( 3*t*t*(1-t) );
                b2Vec2 pd = v4;
                pd *= ( t*t*t );
                vs[i] = pa+pb+pc+pd;
                t+=step;
                }

                vs[21].Set(11,25);

                b2ChainShape boxShape;
                boxShape.CreateChain(vs, times);

                b2FixtureDef boxFixtureDef;
                boxFixtureDef.shape = &boxShape;
                boxFixtureDef.density = 1;

                b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
                staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

        // Left Part obstacles above rotators
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(0, 10); //slightly lower position
            {
                int times = 22;//set the initial value
                b2Vec2 vs[times];

                float step = 1/(float)10;
                float t = 0;

                b2Vec2 v1;
                v1.Set(-17,20);
                b2Vec2 v2;
                v2.Set(-16,15);
                b2Vec2 v3;
                v3.Set(-15,14);
                b2Vec2 v4;
                v4.Set(-14,14);

                //vs[0].Set(17,-14);

                for(int i = 0;i < 10;i++)
                {
                b2Vec2 pa = v1;
                pa *= ( (1-t)*(1-t)*(1-t) );
                b2Vec2 pb = v2;
                pb *= ( 3*t*(1-t)*(1-t) );
                b2Vec2 pc = v3;
                pc *= ( 3*t*t*(1-t) );
                b2Vec2 pd = v4;
                pd *= ( t*t*t );
                vs[i] = pa+pb+pc+pd;
                t+=step;
                }

                vs[10].Set(-14,14);
                //vs[11].Set(10,16);

                step = 1/(float)10;
                t = 0;

                v1.Set(-12,16);
                v2.Set(-16,18);
                v3.Set(-16,19);
                v4.Set(-17,20);

                for(int i = 11;i < 21;i++)
                {
                b2Vec2 pa = v1;
                pa *= ( (1-t)*(1-t)*(1-t) );
                b2Vec2 pb = v2;
                pb *= ( 3*t*(1-t)*(1-t) );
                b2Vec2 pc = v3;
                pc *= ( 3*t*t*(1-t) );
                b2Vec2 pd = v4;
                pd *= ( t*t*t );
                vs[i] = pa+pb+pc+pd;
                t+=step;
                }

                vs[21].Set(-17,20);

                b2ChainShape boxShape;
                boxShape.CreateChain(vs, times);

                b2FixtureDef boxFixtureDef;
                boxFixtureDef.shape = &boxShape;
                boxFixtureDef.density = 1;

                b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
                staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

        // Red Part obstacles for hole
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(0, 10); //slightly lower position
            {
                int times = 14;//set the initial value
                b2Vec2 vs[times];

                float step = 1/(float)10;
                float t = 0;

                b2Vec2 v1;
                v1.Set(14,12);
                b2Vec2 v2;
                v2.Set(13.5,11);
                b2Vec2 v3;
                v3.Set(13,9);
                b2Vec2 v4;
                v4.Set(12.5,9);

                //vs[0].Set(17,-14);

                for(int i = 0;i < 10;i++)
                {
                b2Vec2 pa = v1;
                pa *= ( (1-t)*(1-t)*(1-t) );
                b2Vec2 pb = v2;
                pb *= ( 3*t*(1-t)*(1-t) );
                b2Vec2 pc = v3;
                pc *= ( 3*t*t*(1-t) );
                b2Vec2 pd = v4;
                pd *= ( t*t*t );
                vs[i] = pa+pb+pc+pd;
                t+=step;
                }

                vs[10].Set(12.5,9);
                vs[11].Set(12.5,5);
                vs[12].Set(14,4);
                vs[13].Set(14,12);

                b2ChainShape boxShape;
                boxShape.CreateChain(vs, times);

                b2FixtureDef boxFixtureDef;
                boxFixtureDef.shape = &boxShape;
                boxFixtureDef.density = 1;

                b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
                staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

        // Top small rectangles
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(6, 36); //slightly lower position

            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.2,1.2);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
        }

        // Top small rectangles
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(3, 36); //slightly lower position

            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.2,1.2);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
        }

        // Top small rectangles
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(0, 36); //slightly lower position

            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.2,1.2);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
        }

        // Sling Shot left
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(0, 10); //slightly lower position
            {
                int times = 4;//set the initial value
                b2Vec2 vs[times];

                vs[0].Set(-9,-3);
                vs[1].Set(-6,-8);
                vs[2].Set(-9,-6);
                vs[3].Set(-9,-3);

                b2ChainShape boxShape;
                boxShape.CreateChain(vs, times);

                b2FixtureDef boxFixtureDef;
                boxFixtureDef.shape = &boxShape;
                boxFixtureDef.density = 1;

                b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
                staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
            b2BodyDef myBodyDef1;
            myBodyDef1.type = b2_staticBody; //this will be a static body
            myBodyDef1.position.Set(0, 10); //slightly lower position
            {
                int times = 4;//set the initial value
                b2Vec2 vs[times];

                vs[0].Set(-8.85,-3.25);
                vs[1].Set(-8.75,-2.75);
                vs[2].Set(-5.75,-7.75);
                vs[3].Set(-6.15,-8);

                b2ChainShape boxShape;
                boxShape.CreateChain(vs, times);

                b2FixtureDef boxFixtureDef;
                boxFixtureDef.shape = &boxShape;
                boxFixtureDef.density = 1;
                boxFixtureDef.restitution = 1.2f;

                b2Body* staticBody = m_world->CreateBody(&myBodyDef1); //add body to world
                staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

        // Sling Shot right
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(0, 10); //slightly lower position
            {
                int times = 4;//set the initial value
                b2Vec2 vs[times];

                vs[0].Set(7,-3);
                vs[1].Set(4,-8);
                vs[2].Set(7,-6);
                vs[3].Set(7,-3);

                b2ChainShape boxShape;
                boxShape.CreateChain(vs, times);

                b2FixtureDef boxFixtureDef;
                boxFixtureDef.shape = &boxShape;
                boxFixtureDef.density = 1;

                b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
                staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
            b2BodyDef myBodyDef1;
            myBodyDef1.type = b2_staticBody; //this will be a static body
            myBodyDef1.position.Set(0, 10); //slightly lower position
            {
                int times = 4;//set the initial value
                b2Vec2 vs[times];

                vs[0].Set(6.85,-3.25);
                vs[1].Set(6.75,-2.75);
                vs[2].Set(3.75,-7.75);
                vs[3].Set(4.15,-8);

                b2ChainShape boxShape;
                boxShape.CreateChain(vs, times);

                b2FixtureDef boxFixtureDef;
                boxFixtureDef.shape = &boxShape;
                boxFixtureDef.density = 1;
                boxFixtureDef.restitution = 1.2f;

                b2Body* staticBody = m_world->CreateBody(&myBodyDef1); //add body to world
                staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

        // Flipper wheel 
        {
            b2PolygonShape shape;
            shape.SetAsBox(2.0f, 0.1f);
          
            b2BodyDef bd;
            bd.position.Set(0.5f, 14.0f);
            bd.type = b2_kinematicBody;
            //b2Body* 
            flipperrotbody1 = m_world->CreateBody(&bd);
            b2FixtureDef *fd = new b2FixtureDef;
            fd->density = 1.f;
            fd->shape = new b2PolygonShape;
            fd->shape = &shape;
            flipperrotbody1->CreateFixture(fd);
            flipperrotbody1->SetAngularVelocity( -3 );


            b2PolygonShape shape2;
            shape2.SetAsBox(0.1f, 2.0f);
            b2BodyDef bd2;
            bd2.position.Set(0.5f, 14.0f);
            bd2.type = b2_kinematicBody;
            //b2Body* 
            flipperrotbody2 = m_world->CreateBody(&bd2);  //here we made a bd2 mistake
            b2FixtureDef *fd2 = new b2FixtureDef;
            fd2->density = 1.f;
            fd2->shape = new b2PolygonShape;
            fd2->shape = &shape2;
            flipperrotbody2->CreateFixture(fd2);
            flipperrotbody2->SetAngularVelocity( -3 );

            b2RevoluteJointDef jointDef;
            jointDef.bodyA = flipperrotbody1;
            jointDef.bodyB = flipperrotbody2;
            jointDef.localAnchorA.Set(0,0);
            jointDef.localAnchorB.Set(0,0);
            jointDef.collideConnected = false;
            m_world->CreateJoint(&jointDef);
        }

        //Launcher initial for the game starting
        {
            b2PolygonShape shape;
            shape.SetAsBox(1.0f, 1.0f);
          
            b2BodyDef bd;
            bd.position.Set(18.5f, -1.0f);
            bd.type = b2_dynamicBody;
            bd.allowSleep = false;
            b2Body* launcher;
            launcher = m_world->CreateBody(&bd);
            b2FixtureDef *fd = new b2FixtureDef;
            fd->density = 1.f;
            fd->shape = new b2PolygonShape;
            fd->shape = &shape;
            launcher->CreateFixture(fd);


            b2PolygonShape shape2;
            shape2.SetAsBox(0.2f, 0.5f);
            b2BodyDef bd2;
            bd2.position.Set(18.5f, -2.5f);
            bd2.type = b2_staticBody;
            bd2.allowSleep = false;
            b2Body* launcherSupport;
            launcherSupport = m_world->CreateBody(&bd2);
            b2FixtureDef *fd2 = new b2FixtureDef;
            fd2->density = 1.f;
            fd2->shape = new b2PolygonShape;
            fd2->shape = &shape2;
            launcherSupport->CreateFixture(fd2);

            //b2PrismaticJointDef launcherJoint;
            launcherJoint.bodyA = launcher;
            launcherJoint.bodyB = launcherSupport;
            launcherJoint.localAnchorA.Set(0,0);
            launcherJoint.localAnchorB.Set(0,1.5);
            //launcherJoint.localAxisA.Set(0,1);
            launcherJoint.referenceAngle = 0.0174532925*90;
            launcherJoint.enableLimit = true;
            launcherJoint.lowerTranslation = 0;
            launcherJoint.upperTranslation = 5;
            launcherJoint.collideConnected = true;
            m_world->CreateJoint( &launcherJoint );          
            //launcherJoint.enableMotor = true;
            //launcherJoint.maxMotorForce = 500;//this is a powerful machine after all...
            //launcherJoint.motorSpeed = 5;//5 units per second in positive axis direction
            //m_world->CreateJoint( &launcherJoint );
        }

        //Launcher inside the board on the right bottom side
        {
            b2PolygonShape shape;
            shape.SetAsBox(1.0f, 1.0f);
          
            b2BodyDef bd;
            bd.position.Set(15.5f, -1.0f);
            bd.type = b2_dynamicBody;
            bd.allowSleep = false;
            b2Body* launcher;
            launcher = m_world->CreateBody(&bd);
            b2FixtureDef *fd = new b2FixtureDef;
            fd->density = 1.f;
            fd->shape = new b2PolygonShape;
            fd->shape = &shape;
            launcher->CreateFixture(fd);


            b2PolygonShape shape2;
            shape2.SetAsBox(0.2f, 0.5f);
            b2BodyDef bd2;
            bd2.position.Set(15.5f, -2.5f);
            bd2.type = b2_staticBody;
            bd2.allowSleep = false;
            b2Body* launcherSupport;
            launcherSupport = m_world->CreateBody(&bd2);
            b2FixtureDef *fd2 = new b2FixtureDef;
            fd2->density = 1.f;
            fd2->shape = new b2PolygonShape;
            fd2->shape = &shape2;
            launcherSupport->CreateFixture(fd2);

            //b2PrismaticJointDef launcherJoint;
            launcherJoint2.bodyA = launcher;
            launcherJoint2.bodyB = launcherSupport;
            launcherJoint2.localAnchorA.Set(0,0);
            launcherJoint2.localAnchorB.Set(0,1.5);
            //launcherJoint.localAxisA.Set(0,1);
            launcherJoint2.referenceAngle = 0.0174532925*90;
            launcherJoint2.enableLimit = true;
            launcherJoint2.lowerTranslation = 0;
            launcherJoint2.upperTranslation = 5;
            launcherJoint2.collideConnected = true;
            m_world->CreateJoint( &launcherJoint2 );
        }
        
        // Yellow Bumper to the right having hole connection
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(12.2, 15.5); //slightly lower position

            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.5,0.3,b2Vec2(0,0),1.56);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.5,0.3,b2Vec2(0,1),1.56);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.5,0.3,b2Vec2(0,2),1.56);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.5,0.3,b2Vec2(0,3),1.56);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

        // Yellow Bumper to the to the red part above rotators
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(-13.5, 24); //slightly lower position

            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.3536,0.3536,b2Vec2(0,0),0.78);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.3536,0.3536,b2Vec2(0.5,0.5),0.78);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.3536,0.3536,b2Vec2(1,1),0.78);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.3536,0.3536,b2Vec2(1.5,1.5),0.78);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

        // Yellow Bumper to the to the red part in the upper part having hole
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(-10.2, 32); //slightly lower position

            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.4507,0.4507,b2Vec2(0,0),0.588);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.4507,0.4507,b2Vec2(0.65,0.4507),0.588);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.4507,0.4507,b2Vec2(1.3,0.9014),0.588);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.4507,0.4507,b2Vec2(1.95,1.3521),0.588);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }

        // Yellow Bumper attached to the circular bumper
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(-1, 22.5); //slightly lower position

            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.3536,0.3536,b2Vec2(0,0),2.34);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.3536,0.3536,b2Vec2(0.5,-0.5),2.34);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.3536,0.3536,b2Vec2(1,-1),2.34);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(0.3536,0.3536,b2Vec2(1.5,-1.5),2.34);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;
            boxFixtureDef.restitution = 1.0f;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
            b2BodyDef myBodyDef1;
            myBodyDef1.type = b2_staticBody; //this will be a static body
            myBodyDef1.position.Set(0, 10); //slightly lower position
            {
                int times = 2;//set the initial value
                b2Vec2 vs1[times];
                b2Vec2 vs2[times];

                vs1[0].Set(-1,13);
                vs1[1].Set(-0.2,14);
                vs2[0].Set(1,11);
                vs2[1].Set(2,13);

                b2ChainShape boxShape;
                boxShape.CreateChain(vs1, times);
                b2ChainShape boxShape1;
                boxShape1.CreateChain(vs2, times);

                b2FixtureDef boxFixtureDef;
                boxFixtureDef.shape = &boxShape;
                boxFixtureDef.density = 1;
                b2FixtureDef boxFixtureDef1;
                boxFixtureDef1.shape = &boxShape1;
                boxFixtureDef1.density = 1;

                b2Body* staticBody = m_world->CreateBody(&myBodyDef1); //add body to world
                staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
                b2Body* staticBody1 = m_world->CreateBody(&myBodyDef1); //add body to world
                staticBody1->CreateFixture(&boxFixtureDef1); //add fixture to body
            }
        }

        
        //ball
        {
            b2BodyDef ballBodyDef;
            ballBodyDef.allowSleep = false;
            ballBodyDef.type = b2_dynamicBody;
            ballBodyDef.position.Set(19.5, 4);
            //ballBodyDef.position.Set(-19.5, 20);
            {
            b2CircleShape ball;
            ball.m_p.Set(0, 0);
            ball.m_radius = 0.5;
            
            b2FixtureDef ballFixtureDef;
            ballFixtureDef.shape = &ball;
            ballFixtureDef.density = 0.05;

            //b2Body* 
            ballBody = m_world->CreateBody(&ballBodyDef);
            ballBody->CreateFixture(&ballFixtureDef);
            }
        }
        
    }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}