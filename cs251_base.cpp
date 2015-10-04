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

#include "cs251_base.hpp"
#include <cstdio>
using namespace std;
using namespace cs251;


base_sim_t::base_sim_t()
{
	b2Vec2 gravity;
	gravity.Set(0.0f, -10.0f);
	m_world = new b2World(gravity);



	m_text_line = 30;

	m_point_count = 0;

	m_world->SetDebugDraw(&m_debug_draw);
	
	m_step_count = 0;

	b2BodyDef body_def;
	m_ground_body = m_world->CreateBody(&body_def);

	memset(&m_max_profile, 0, sizeof(b2Profile));
	memset(&m_total_profile, 0, sizeof(b2Profile));


}

void base_sim_t::launch()
        {
          /*          
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

          m_world->CreateJoint( &prismaticJointDef );*/
        {
          extern b2PrismaticJointDef launcherJoint;
          //launcherJoint.referenceAngle = 0.0174532925*90;
          //launcherJoint.localAxisA.Set(0,1);
          launcherJoint.enableLimit = true;
          launcherJoint.lowerTranslation = 0;
          launcherJoint.upperTranslation = 2;
          launcherJoint.enableMotor = true;
          launcherJoint.maxMotorForce = 500000;//this is a powerful machine after all...
          launcherJoint.motorSpeed = 50;//5 units per second in positive axis direction
          
          m_world->CreateJoint(&launcherJoint);
          //launcherJoint.motorSpeed = -5;
          //m_world->CreateJoint(&launcherJoint);
          launcherJoint.enableMotor = false;          
        }



        }

void base_sim_t::delaunch()
{
  {
          extern b2PrismaticJointDef launcherJoint;
          //launcherJoint.referenceAngle = 0.0174532925*90;
          //launcherJoint.localAxisA.Set(0,1);
          launcherJoint.enableLimit = true;
          launcherJoint.lowerTranslation = 0;
          launcherJoint.upperTranslation = 2;
          launcherJoint.enableMotor = true;
          launcherJoint.maxMotorForce = 500000;//this is a powerful machine after all...
          launcherJoint.motorSpeed = -50;//5 units per second in positive axis direction
          
          m_world->CreateJoint(&launcherJoint);
          //launcherJoint.motorSpeed = -5;
          //m_world->CreateJoint(&launcherJoint);
          launcherJoint.enableMotor = false;
  }
}

void base_sim_t::launch2()
        {
          {
            extern b2PrismaticJointDef launcherJoint2;
            launcherJoint2.enableLimit = true;
            launcherJoint2.lowerTranslation = 0;
            launcherJoint2.upperTranslation = 0.7;
            launcherJoint2.enableMotor = true;
            launcherJoint2.maxMotorForce = 5000;//this is a powerful machine after all...
            launcherJoint2.motorSpeed = 32;//5 units per second in positive axis direction
            
            m_world->CreateJoint(&launcherJoint2);
            launcherJoint2.enableMotor = false;          
          }
        }

void base_sim_t::delaunch2()
{
  {
          extern b2PrismaticJointDef launcherJoint2;
          launcherJoint2.enableLimit = true;
          launcherJoint2.lowerTranslation = 0;
          launcherJoint2.upperTranslation = 0.7;
          launcherJoint2.enableMotor = true;
          launcherJoint2.maxMotorForce = 5000;//this is a powerful machine after all...
          launcherJoint2.motorSpeed = -32;//5 units per second in positive axis direction
          
          m_world->CreateJoint(&launcherJoint2);
          launcherJoint2.enableMotor = false;
  }
}

void base_sim_t::ball()
{
    //extern b2BodyDef ballBodyDef;
    //extern b2FixtureDef ballFixtureDef;
    extern b2Body* ballBody;
    ballBody->SetTransform(b2Vec2(10,20), 0);
    //m_world->DestroyBody(ballBody); //this is to delete the ball
    //ballBody = m_world->CreateBody(&ballBodyDef); //add body to world
    //ballBody->CreateFixture(&ballFixtureDef); //add fixture to body
    //delete ballBody;
}

/*void base_sim_t::createball()
{
    extern b2BodyDef ballBodyDef;
    extern b2FixtureDef ballFixtureDef;
    extern b2Body* ballBody;
    //m_world->DestroyBody(ballBody);
    ballBody = m_world->CreateBody(&ballBodyDef); //add body to world
    ballBody->CreateFixture(&ballFixtureDef); //add fixture to body
}*/

void base_sim_t::flipperleft()
{
    extern b2Body* flipperleftbody;
    //flipperleftbody->SetLinearVelocity( b2Vec2(0,10) );
    flipperleftbody->ApplyLinearImpulse( b2Vec2(0,50), flipperleftbody->GetWorldCenter(), 1 );
    
    //check for coming down slowly
    //flipperleftbody->ApplyAngularImpulse( 10 , 1);
    /*extern b2Body* flipperwheelrightbody;
    //b2RevoluteJointDef wheeljointDef1;
    flipperwheelrightbody->ApplyLinearImpulse( b2Vec2(0,-50), flipperwheelrightbody->GetWorldCenter(), 1 );*/

}

void base_sim_t::flipperright()
{
    extern b2Body* flipperrightbody;
    flipperrightbody->ApplyLinearImpulse( b2Vec2(0,50), flipperrightbody->GetWorldCenter(), 1 );
    /*extern b2Body* flipperwheelrightbody;
    //b2RevoluteJointDef wheeljointDef1;
    flipperwheelrightbody->ApplyLinearImpulse( b2Vec2(0,50), flipperwheelrightbody->GetWorldCenter(), 1 );
    extern b2Body* flipperwheelupbody;
    //b2RevoluteJointDef wheeljointDef1;
    flipperwheelupbody->ApplyLinearImpulse( b2Vec2(50,0), flipperwheelupbody->GetWorldCenter(), 1 );
    //flipperwheelupbody->ApplyLinearImpulse( b2Vec2(0,50), flipperwheelupbody->GetWorldCenter(), 1 );        
    extern b2Body* flipperwheelleftbody;
    flipperwheelleftbody->ApplyLinearImpulse( b2Vec2(0,-50), flipperwheelleftbody->GetWorldCenter(), 1 );
    extern b2Body* flipperwheeldownbody;
    flipperwheeldownbody->ApplyLinearImpulse( b2Vec2(-50,0), flipperwheeldownbody->GetWorldCenter(), 1 );*/

}

void base_sim_t::flipperwheelright()
{
    extern b2Body* flipperrotbody1;
    flipperrotbody1->SetAngularVelocity( -5 );
    extern b2Body* flipperrotbody2;
    flipperrotbody2->SetAngularVelocity( -5 );
}

void base_sim_t::flipperwheelleft()
{
    extern b2Body* flipperrotbody1;
    flipperrotbody1->SetAngularVelocity( 5 );
    extern b2Body* flipperrotbody2;
    flipperrotbody2->SetAngularVelocity( 5 );
}

base_sim_t::~base_sim_t()
{
	// By deleting the world, we delete the bomb, mouse joint, etc.
	delete m_world;
	m_world = NULL;
}

void base_sim_t::pre_solve(b2Contact* contact, const b2Manifold* oldManifold)
{
  const b2Manifold* manifold = contact->GetManifold();
  
  if (manifold->pointCount == 0)
    {
      return;
    }
  
  b2Fixture* fixtureA = contact->GetFixtureA();
  b2Fixture* fixtureB = contact->GetFixtureB();
  
  b2PointState state1[b2_maxManifoldPoints], state2[b2_maxManifoldPoints];
  b2GetPointStates(state1, state2, oldManifold, manifold);
  
  b2WorldManifold world_manifold;
  contact->GetWorldManifold(&world_manifold);
  
  for (int32 i = 0; i < manifold->pointCount && m_point_count < k_max_contact_points; ++i)
    {
      contact_point_t* cp = m_points + m_point_count;
      cp->fixtureA = fixtureA;
      cp->fixtureB = fixtureB;
      cp->position = world_manifold.points[i];
      cp->normal = world_manifold.normal;
      cp->state = state2[i];
      ++m_point_count;
    }
}

void base_sim_t::draw_title(int x, int y, const char *string)
{
    m_debug_draw.DrawString(x, y, string);
}

void base_sim_t::step(settings_t* settings)
{
  float32 time_step = settings->hz > 0.0f ? 1.0f / settings->hz : float32(0.0f);

  if (settings->pause)
    {
      if (settings->single_step)
	{
	  settings->single_step = 0;
	}
      else
	{
	  time_step = 0.0f;
	}
      
      m_debug_draw.DrawString(5, m_text_line, "****PAUSED****");
      m_text_line += 15;
    }
  
  uint32 flags = 0;
  flags += settings->draw_shapes			* b2Draw::e_shapeBit;
  flags += settings->draw_joints			* b2Draw::e_jointBit;
  flags += settings->draw_AABBs			* b2Draw::e_aabbBit;
  flags += settings->draw_pairs			* b2Draw::e_pairBit;
  flags += settings->draw_COMs				* b2Draw::e_centerOfMassBit;
  m_debug_draw.SetFlags(flags);
  
  m_world->SetWarmStarting(settings->enable_warm_starting > 0);
  m_world->SetContinuousPhysics(settings->enable_continuous > 0);
  m_world->SetSubStepping(settings->enable_sub_stepping > 0);
  
  m_point_count = 0;
  
  m_world->Step(time_step, settings->velocity_iterations, settings->position_iterations);
  
  m_world->DrawDebugData();
  
  if (time_step > 0.0f)
    {
      ++m_step_count;
    }
  
  if (settings->draw_stats)
    {
      int32 body_count = m_world->GetBodyCount();
      int32 contact_count = m_world->GetContactCount();
      int32 joint_count = m_world->GetJointCount();
      m_debug_draw.DrawString(5, m_text_line, "bodies/contacts/joints = %d/%d/%d", body_count, contact_count, joint_count);
      m_text_line += 15;
      
      int32 proxy_count = m_world->GetProxyCount();
      int32 height = m_world->GetTreeHeight();
      int32 balance = m_world->GetTreeBalance();
      float32 quality = m_world->GetTreeQuality();
      m_debug_draw.DrawString(5, m_text_line, "proxies/height/balance/quality = %d/%d/%d/%g", proxy_count, height, balance, quality);
      m_text_line += 15;
    }
  
  // Track maximum profile times
  {
    const b2Profile& p = m_world->GetProfile();
    m_max_profile.step = b2Max(m_max_profile.step, p.step);
    m_max_profile.collide = b2Max(m_max_profile.collide, p.collide);
    m_max_profile.solve = b2Max(m_max_profile.solve, p.solve);
    m_max_profile.solveInit = b2Max(m_max_profile.solveInit, p.solveInit);
    m_max_profile.solveVelocity = b2Max(m_max_profile.solveVelocity, p.solveVelocity);
    m_max_profile.solvePosition = b2Max(m_max_profile.solvePosition, p.solvePosition);
    m_max_profile.solveTOI = b2Max(m_max_profile.solveTOI, p.solveTOI);
    m_max_profile.broadphase = b2Max(m_max_profile.broadphase, p.broadphase);
    
    m_total_profile.step += p.step;
    m_total_profile.collide += p.collide;
    m_total_profile.solve += p.solve;
    m_total_profile.solveInit += p.solveInit;
    m_total_profile.solveVelocity += p.solveVelocity;
    m_total_profile.solvePosition += p.solvePosition;
    m_total_profile.solveTOI += p.solveTOI;
    m_total_profile.broadphase += p.broadphase;
  }
  
  if (settings->draw_profile)
    {
      const b2Profile& p = m_world->GetProfile();
      
      b2Profile ave_profile;
      memset(&ave_profile, 0, sizeof(b2Profile));
      if (m_step_count > 0)
	{
	  float32 scale = 1.0f / m_step_count;
	  ave_profile.step = scale * m_total_profile.step;
	  ave_profile.collide = scale * m_total_profile.collide;
	  ave_profile.solve = scale * m_total_profile.solve;
	  ave_profile.solveInit = scale * m_total_profile.solveInit;
	  ave_profile.solveVelocity = scale * m_total_profile.solveVelocity;
	  ave_profile.solvePosition = scale * m_total_profile.solvePosition;
	  ave_profile.solveTOI = scale * m_total_profile.solveTOI;
	  ave_profile.broadphase = scale * m_total_profile.broadphase;
	}
      
      m_debug_draw.DrawString(5, m_text_line, "step [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.step, ave_profile.step, m_max_profile.step);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "collide [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.collide, ave_profile.collide, m_max_profile.collide);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solve [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solve, ave_profile.solve, m_max_profile.solve);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solve init [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveInit, ave_profile.solveInit, m_max_profile.solveInit);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solve velocity [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveVelocity, ave_profile.solveVelocity, m_max_profile.solveVelocity);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solve position [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solvePosition, ave_profile.solvePosition, m_max_profile.solvePosition);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solveTOI [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveTOI, ave_profile.solveTOI, m_max_profile.solveTOI);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "broad-phase [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.broadphase, ave_profile.broadphase, m_max_profile.broadphase);
      m_text_line += 15;
    }
    
  if (settings->draw_contact_points)
    {
      //const float32 k_impulseScale = 0.1f;
      const float32 k_axis_scale = 0.3f;
      
      for (int32 i = 0; i < m_point_count; ++i)
	{
	  contact_point_t* point = m_points + i;
	  
	  if (point->state == b2_addState)
	    {
	      // Add
	      m_debug_draw.DrawPoint(point->position, 10.0f, b2Color(0.3f, 0.95f, 0.3f));
	    }
	  else if (point->state == b2_persistState)
	    {
	      // Persist
	      m_debug_draw.DrawPoint(point->position, 5.0f, b2Color(0.3f, 0.3f, 0.95f));
	    }
	  
	  if (settings->draw_contact_normals == 1)
	    {
	      b2Vec2 p1 = point->position;
	      b2Vec2 p2 = p1 + k_axis_scale * point->normal;
	      m_debug_draw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.9f));
	    }
	  else if (settings->draw_contact_forces == 1)
	    {
	      //b2Vec2 p1 = point->position;
	      //b2Vec2 p2 = p1 + k_forceScale * point->normalForce * point->normal;
	      //DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
	    }
	  
	  if (settings->draw_friction_forces == 1)
	    {
	      //b2Vec2 tangent = b2Cross(point->normal, 1.0f);
	      //b2Vec2 p1 = point->position;
	      //b2Vec2 p2 = p1 + k_forceScale * point->tangentForce * tangent;
	      //DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
	    }
	}
    }
}
