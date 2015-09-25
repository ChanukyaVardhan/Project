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
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   */ 
          

    b2PrismaticJointDef launcherJoint;
    b2Body* ballBody;
    b2Body* flipperleftbody;
    b2BodyDef ballBodyDef;
    b2FixtureDef ballFixtureDef;
    dominos_t::dominos_t()
    {
        //Ground
        /*! \var b1 
         * \brief pointer to the body ground 
         */ 
        /*  //This is our outer box  and three rectangles
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(0, 10); //slightly lower position

            b2PolygonShape boxShape;
            boxShape.SetAsBox(1,1);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
        }*/

        /*  This are our circles
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(0, 10); //slightly lower position
            {
            b2CircleShape circleShape1;
            circleShape1.m_p.Set(0, 0); //position, relative to body position
            circleShape1.m_radius = 5; //radius
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &circleShape1;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }

            {
            b2CircleShape circleShape2;
            circleShape2.m_p.Set(0, 0); //position, relative to body position
            circleShape2.m_radius = 2; //radius
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &circleShape2;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }*/

        /* this is red part with hole but no hole
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(0, 10); //slightly lower position

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
        }*/

        /* Yellow Bumpers
        {
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(0, 10); //slightly lower position

            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(1,1,b2Vec2(0,0),0.78);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(1,1,b2Vec2(1.414,1.414),0.78);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(1,1,b2Vec2(2.828,2.828),0.78);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
            {
            b2PolygonShape boxShape;
            boxShape.SetAsBox(1,1,b2Vec2(4.242,4.242),0.78);
              
            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 1;

            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
            }
        }*/

        /* curve red on right side and on the left side also and even the remaining red colors
        {
        b2BodyDef myBodyDef;
        myBodyDef.type = b2_staticBody; //this will be a static body
        myBodyDef.position.Set(0, 10); //slightly lower position

        b2Vec2 temp1,temp2;

        int times = 15;
        {
        b2Vec2 vs[times];
        
        float step = 1/(float)times;
        float t = 0;

        b2Vec2 v1;
        v1.Set(1,3);
        b2Vec2 v2;
        v2.Set(3,3);
        b2Vec2 v3;
        v3.Set(5,-5);
        b2Vec2 v4;
        v4.Set(-3,-5);

        for(int i = 0;i < times;i++)
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
        temp1 = vs[times-1];

        b2ChainShape boxShape;
        boxShape.CreateChain(vs, times);

        b2FixtureDef boxFixtureDef;
        boxFixtureDef.shape = &boxShape;
        boxFixtureDef.density = 1;

        b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
        staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
        //staticBody->SetTransform( staticBody->GetPosition(), -0.78 );
        }
        {
        b2Vec2 vs[times];
        
        float step = 1/(float)times;
        float t = 0;

        b2Vec2 v1;
        v1.Set(1,3);
        b2Vec2 v2;
        v2.Set(2,3);
        b2Vec2 v3;
        v3.Set(4,-5);
        b2Vec2 v4;
        v4.Set(-5,-5);

        for(int i = 0;i < times;i++)
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
        temp2 = vs[times-1];

        b2ChainShape boxShape;
        boxShape.CreateChain(vs, times);

        b2FixtureDef boxFixtureDef;
        boxFixtureDef.shape = &boxShape;
        boxFixtureDef.density = 1;

        b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
        staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
        //staticBody->SetTransform( staticBody->GetPosition(), -0.78 );
        }
        {
        b2Vec2 vs[2];
        vs[0] = temp1;
        vs[1] = temp2;
        b2ChainShape boxShape;
        boxShape.CreateChain(vs, 2);

        b2FixtureDef boxFixtureDef;
        boxFixtureDef.shape = &boxShape;
        boxFixtureDef.density = 1;

        b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
        staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
        //staticBody->SetTransform( staticBody->GetPosition(), -0.78 );
        }
        }*/

        /* This is for rotators
        {
        
        b2PolygonShape shape;
        shape.SetAsBox(2.2f, 0.2f);
      
        b2BodyDef bd;
        bd.position.Set(14.0f, 14.0f);
        bd.type = b2_kinematicBody;
        b2Body* body = m_world->CreateBody(&bd);
        b2FixtureDef *fd = new b2FixtureDef;
        fd->density = 1.f;
        fd->shape = new b2PolygonShape;
        fd->shape = &shape;
        body->CreateFixture(fd);
        body->SetAngularVelocity( 5 );


        b2PolygonShape shape2;
        shape2.SetAsBox(0.2f, 2.0f);
        b2BodyDef bd2;
        bd2.position.Set(14.0f, 14.0f);
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

        }*/
//The world
    b2Body* b1;  
    {
      
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }

    /* //This is for the flipper bats
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
  
      b2BodyDef flipper;
      flipper.position.Set(14.0f, 14.0f);
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
      bd2.position.Set(14.0f, 16.0f);
      bd2.allowSleep = false;
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = flipperleftbody;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(-2.2,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      jointDef.enableLimit = true;
      jointDef.lowerAngle = -30 * 0.0174532925;
      jointDef.upperAngle =  30 * 0.0174532925;
      m_world->CreateJoint(&jointDef);
    }*/

     //ball
    {
        b2BodyDef ballBodyDef;
        ballBodyDef.type = b2_dynamicBody; //this will be a static body
        ballBodyDef.position.Set(0, 10); //slightly lower position
        {
        b2CircleShape ball;
        ball.m_p.Set(0, 0); //position, relative to body position
        ball.m_radius = 2; //radius
          
        b2FixtureDef ballFixtureDef;
        ballFixtureDef.shape = &ball;
        ballFixtureDef.density = 1;

        //b2Body* 
        ballBody = m_world->CreateBody(&ballBodyDef); //add body to world
        ballBody->CreateFixture(&ballFixtureDef); //add fixture to body
        //m_world->DestroyBody(ballBody);
        }
    }
        /*No need of this
        {
            if(x==1)
            {
                b2PolygonShape shape;
        shape.SetAsBox(2.2f, 0.2f);
      
        b2BodyDef bd;
        bd.position.Set(14.0f, 24.0f);
        bd.type = b2_dynamicBody;
        b2Body* body = m_world->CreateBody(&bd);
        b2FixtureDef *fd = new b2FixtureDef;
        fd->density = 1.f;
        fd->shape = new b2PolygonShape;
        fd->shape = &shape;
        body->CreateFixture(fd);


        b2PolygonShape shape2;
        shape2.SetAsBox(0.2f, 2.0f);
        b2BodyDef bd2;
        bd2.position.Set(14.0f, 10.0f);
        bd2.type = b2_dynamicBody;
        b2Body* body2 = m_world->CreateBody(&bd2);
        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->density = 1.f;
        fd2->shape = new b2PolygonShape;
        fd2->shape = &shape2;
        body2->CreateFixture(fd2);



          b2PrismaticJointDef prismaticJointDef;
          prismaticJointDef.bodyA = body;
          prismaticJointDef.bodyB = body2;
          prismaticJointDef.localAnchorA.Set(0,0);
          prismaticJointDef.localAnchorB.Set(0,14);
          prismaticJointDef.referenceAngle = 0.0174532925*90;
          prismaticJointDef.enableLimit = true;
          prismaticJointDef.lowerTranslation = 0;
          prismaticJointDef.upperTranslation = 5;
          prismaticJointDef.collideConnected = true;
          prismaticJointDef.enableMotor = true;
          prismaticJointDef.maxMotorForce = 500;//this is a powerful machine after all...
          prismaticJointDef.motorSpeed = 5;//5 units per second in positive axis direction
          m_world->CreateJoint( &prismaticJointDef );
            }
        }*/

        /*{
            b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;
  b2FixtureDef fixtureDef;
  fixtureDef.density = 1;
  
  //two boxes
  b2PolygonShape squareShapeA;
  squareShapeA.SetAsBox(5,3);
  b2PolygonShape squareShapeB;
  squareShapeB.SetAsBox(1,4);
  
  //large box a little to the left
  bodyDef.position.Set(-20, 10);
  fixtureDef.shape = &squareShapeA;
  b2Body* m_bodyA = m_world->CreateBody(&bodyDef);
  m_bodyA = m_world->CreateBody( &bodyDef );
  m_bodyA->CreateFixture( &fixtureDef );
  
  //smaller box a little to the right
  bodyDef.position.Set( -4, 10);
  fixtureDef.shape = &squareShapeB;
  b2Body* m_bodyB = m_world->CreateBody(&bodyDef);
  m_bodyB = m_world->CreateBody( &bodyDef );
  m_bodyB->CreateFixture( &fixtureDef );

  b2PrismaticJointDef prismaticJointDef;
  prismaticJointDef.bodyA = m_bodyA;
  prismaticJointDef.bodyB = m_bodyB;
  prismaticJointDef.localAnchorA.Set( 6,-3);//a little outside the bottom right corner
  prismaticJointDef.localAnchorB.Set(-1,-4);//bottom left corner
  prismaticJointDef.enableLimit = true;
  prismaticJointDef.lowerTranslation = 0;
  prismaticJointDef.upperTranslation = 10;
  prismaticJointDef.collideConnected = true;

  m_world->CreateJoint( &prismaticJointDef );

        }*/
  /* //This is the s function part in dominos
  {
        b2PolygonShape shape;
        shape.SetAsBox(2.0f, 2.0f);
      
        b2BodyDef bd;
        bd.position.Set(14.0f, 24.0f);
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
        shape2.SetAsBox(0.2f, 2.0f);
        b2BodyDef bd2;
        bd2.position.Set(14.0f, 10.0f);
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
          launcherJoint.localAnchorB.Set(0,14);
          //launcherJoint.localAxisA.Set(0,1);
          launcherJoint.referenceAngle = 0.0174532925*90;
          //launcherJoint.enableLimit = true;
          //launcherJoint.lowerTranslation = 0;
          //launcherJoint.upperTranslation = 5;
          launcherJoint.collideConnected = true;
          //m_world->CreateJoint( &launcherJoint );          
          //launcherJoint.enableMotor = true;
          //launcherJoint.maxMotorForce = 500;//this is a powerful machine after all...
          //launcherJoint.motorSpeed = 5;//5 units per second in positive axis direction
          //m_world->CreateJoint( &launcherJoint );
  }*/
        /*{
            b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
    
      b2BodyDef bd;
      bd.position.Set(14.0f, 14.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(14.0f, 16.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
        }*/

    /*This part is for the walls that fix flipper bats still work is needed to be done in adding the flipper bat
    {
    b2BodyDef myBodyDef;
    myBodyDef.type = b2_staticBody; //this will be a static body
    myBodyDef.position.Set(0, 10); //slightly lower position

    b2Vec2 vertices[4];
    vertices[0].Set(   0,   10);
    vertices[1].Set( 0.5,   10);
    vertices[2].Set( 0.5,    0);
    //vertices[3].Set(  -5,   -5);
    //vertices[4].Set(-5.5, -4.5);
    vertices[3].Set(   0,    1);

    b2PolygonShape boxShape;
    boxShape.Set(vertices,4);
      
    b2FixtureDef boxFixtureDef;
    boxFixtureDef.shape = &boxShape;
    boxFixtureDef.density = 1;

    b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
    staticBody->CreateFixture(&boxFixtureDef); //add fixture to body

    b2Vec2 vertices2[4];
    //vertices[0].Set(   0,   10);
    //vertices[1].Set( 0.5,   10);
    vertices2[0].Set( 0.5,    0);
    vertices2[1].Set(  -5,   -5);
    vertices2[2].Set(-5.5, -4.5);
    vertices2[3].Set(   0,    1);

    b2PolygonShape boxShape2;
    boxShape2.Set(vertices2,4);
      
    b2FixtureDef boxFixtureDef2;
    boxFixtureDef2.shape = &boxShape2;
    boxFixtureDef2.density = 1;

    b2Body* staticBody2 = m_world->CreateBody(&myBodyDef); //add body to world
    staticBody2->CreateFixture(&boxFixtureDef2); //add fixture to body
    }*/

    
      /*{
      b2BodyDef myBodyDef;
      myBodyDef.type = b2_staticBody; //this will be a static body
      myBodyDef.position.Set(0, 10); //slightly lower position

      b2Vec2 temp1,temp2;

      int times = 15;
      {
      b2Vec2 vs[times];
      
      float step = 1/(float)times;
      float t = 0;

      b2Vec2 v1;
      v1.Set(1,3);
      b2Vec2 v2;
      v2.Set(3,3);
      b2Vec2 v3;
      v3.Set(5,-5);
      b2Vec2 v4;
      v4.Set(-3,-5);

      for(int i = 0;i < times;i++)
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
      temp1 = vs[times-1];

      b2ChainShape boxShape;
      boxShape.CreateChain(vs, times);

      b2FixtureDef boxFixtureDef;
      boxFixtureDef.shape = &boxShape;
      boxFixtureDef.density = 1;

      b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
      staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
      //staticBody->SetTransform( staticBody->GetPosition(), -0.78 );
      }
      {
      b2Vec2 vs[times];
      
      float step = 1/(float)times;
      float t = 0;

      b2Vec2 v1;
      v1.Set(1,3);
      b2Vec2 v2;
      v2.Set(2,3);
      b2Vec2 v3;
      v3.Set(4,-5);
      b2Vec2 v4;
      v4.Set(-5,-5);

      for(int i = 0;i < times;i++)
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
      temp2 = vs[times-1];

      b2ChainShape boxShape;
      boxShape.CreateChain(vs, times);

      b2FixtureDef boxFixtureDef;
      boxFixtureDef.shape = &boxShape;
      boxFixtureDef.density = 1;

      b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
      staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
      //staticBody->SetTransform( staticBody->GetPosition(), -0.78 );
      }
      {
      b2Vec2 vs[2];
      vs[0] = temp1;
      vs[1] = temp2;
      b2ChainShape boxShape;
      boxShape.CreateChain(vs, 2);

      b2FixtureDef boxFixtureDef;
      boxFixtureDef.shape = &boxShape;
      boxFixtureDef.density = 1;

      b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
      staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
      //staticBody->SetTransform( staticBody->GetPosition(), -0.78 );
      }
      }*/

      /* This is for rotators
      {
      
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
    
      b2BodyDef bd;
      bd.position.Set(14.0f, 14.0f);
      bd.type = b2_kinematicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);
      body->SetAngularVelocity( 5 );


      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(14.0f, 14.0f);
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

        }*/

    }
    
    


  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}



/*{
            //body and fixture defs - the common parts
  b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;
  b2FixtureDef fixtureDef;
  fixtureDef.density = 1;
  
  //two shapes
  b2PolygonShape boxShape;
  boxShape.SetAsBox(2,2);
  b2CircleShape circleShape;
  circleShape.m_radius = 2;     
  
  //make box a little to the left
  bodyDef.position.Set(-3, 10);
  fixtureDef.shape = &boxShape;
  b2Body* m_bodyA = m_world->CreateBody(&bodyDef);
  m_bodyA = m_world->CreateBody( &bodyDef );
  m_bodyA->CreateFixture( &fixtureDef );
  m_bodyA->SetAngularVelocity( 10 );
  
  //and circle a little to the right
  bodyDef.position.Set( 3, 10);
  fixtureDef.shape = &circleShape;
    b2Body* m_bodyB = m_world->CreateBody(&bodyDef);
  m_bodyB = m_world->CreateBody( &bodyDef );
  m_bodyB->CreateFixture( &fixtureDef );
  m_bodyB->SetAngularVelocity( 10 );

  b2RevoluteJointDef revoluteJointDef;
  revoluteJointDef.bodyA = m_bodyA;
  revoluteJointDef.bodyB = m_bodyB;
  revoluteJointDef.collideConnected = false;
  revoluteJointDef.localAnchorA.Set(2,2);//the top right corner of the box
  revoluteJointDef.localAnchorB.Set(0,0);//center of the circle
  revoluteJointDef.enableMotor = true;
  revoluteJointDef.maxMotorTorque = 5;
  revoluteJointDef.motorSpeed = 90 * 0.017;//90 degrees per second
  m_world->CreateJoint( &revoluteJointDef );
        }*/