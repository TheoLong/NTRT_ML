/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

/**
 * @file FlemonsSpineModelContact.cpp
 * @brief Implementing the superhedral complex spine inspired by Tom Flemons
 * @author Brian Mirletz
 * @date November 2014
 * @version 1.0.0
 * $Id$
 */

// This module
#include "SuperballModelContact.h"
// This library
#include "core/tgCast.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "tgcreator/tgBasicContactCableInfo.h"
#include "tgcreator/tgKinematicContactCableInfo.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgKinematicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgUtil.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <algorithm> // std::fill
#include <iostream>
#include <map>
#include <set>

SuperballModelContact::SuperballModelContact(int segments) : 
    BaseSpineModelLearning(segments) 
{
}

SuperballModelContact::~SuperballModelContact()
{
}

void SuperballModelContact::setup(tgWorld& world)
{
    // This is basically a manual setup of a model.
    // There are things that do this for us
    /// @todo: reference the things that do this for us

    // Rod and Muscle configuration
    // Note: This needs to be high enough or things fly apart...
    const double density = 4.2/300.0;
    const double radius  = 0.5;
    const double friction = 0.5;
    const double rollFriction = 0.0;
    const double restitution = 0.0;
    const double rod_length = 20.0;
    const double rod_space = 5.0; 
    const double half_length = rod_length / 2;
    const tgRod::Config rodConfig(radius, density, friction, rollFriction, restitution);
    
    const double elasticity = 1000.0;
    const double damping = 10.0;
    const double pretension = 0.0;
    const bool   history = false;
    const double maxTens = 7000.0;
    const double maxSpeed = 12.0;

    const double mRad = 1.0;
    const double motorFriction = 10.0;
    const double motorInertia = 1.0;
    const bool backDrivable = false;
    tgKinematicActuator::Config motorConfig(elasticity, damping, pretension,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed);
    
    //Node numbers seen from Front
// -----0-------1------
// ---------2----------
// 3------------------4
// ---------5----------
// -----6-------7------
//
//Node numbers seen from Back
// -----0-------1------
// ---------8----------
// 9-----------------10
// ---------11---------
// -----6-------7------
//
    //Create the super structure
    tgStructure super;

    // Nodes for struts
    super.addNode(-rod_space,  -half_length, 0);            // 0
    super.addNode(-rod_space,   half_length, 0);            // 1
    super.addNode( rod_space,  -half_length, 0);            // 2
    super.addNode( rod_space,   half_length, 0);            // 3
    super.addNode(0,           -rod_space,   -half_length); // 4
    super.addNode(0,           -rod_space,    half_length); // 5
    super.addNode(0,            rod_space,   -half_length); // 6
    super.addNode(0,            rod_space,    half_length); // 7
    super.addNode(-half_length, 0,            rod_space);   // 8
    super.addNode( half_length, 0,            rod_space);   // 9
    super.addNode(-half_length, 0,           -rod_space);   // 10
    super.addNode( half_length, 0,           -rod_space);   // 11

    //Node for payload
    //super.addNode(0,0,0, "payload"); //12 -- Dawn's note: I suspect this could be the source of the problem; a node that is not part of a rigid body. May need to make a box here instead, or a short rod, or something else.
    super.addNode(0,0,1, "payload"); //12
    super.addNode(0,0,-1, "payload"); //13

    // Struts
    super.addPair( 0,  1, "rod");
    super.addPair( 2,  3, "rod");
    super.addPair( 4,  5, "rod");
    super.addPair( 6,  7, "rod");
    super.addPair( 8,  9, "rod");
    super.addPair(10, 11, "rod");
    super.addPair(12, 13, "rod"); // The "payload", for now. Has to be a solid rigid body.

    // muscles
    super.addPair(0, 4,  "muscle");
    super.addPair(0, 5,  "muscle");
    super.addPair(0, 8,  "muscle");
    super.addPair(0, 10, "muscle");

    super.addPair(1, 6,  "muscle");
    super.addPair(1, 7,  "muscle");
    super.addPair(1, 8,  "muscle");
    super.addPair(1, 10, "muscle");

    super.addPair(2, 4,  "muscle");
    super.addPair(2, 5,  "muscle");
    super.addPair(2, 9,  "muscle");
    super.addPair(2, 11, "muscle");

    super.addPair(3, 7,  "muscle");
    super.addPair(3, 6,  "muscle");
    super.addPair(3, 9,  "muscle");
    super.addPair(3, 11, "muscle");

    super.addPair(4, 10, "muscle");
    super.addPair(4, 11, "muscle");

    super.addPair(5, 8,  "muscle");
    super.addPair(5, 9,  "muscle");

    super.addPair(6, 10, "muscle");
    super.addPair(6, 11, "muscle");

    super.addPair(7, 8,  "muscle");
    super.addPair(7, 9,  "muscle");

    // Payload Muscles
    super.addPair(0, 12, "muscle");
    super.addPair(1, 12, "muscle");
    super.addPair(2, 12, "muscle");
    super.addPair(3, 12, "muscle");
    super.addPair(4, 12, "muscle");
    super.addPair(5, 12, "muscle");
    super.addPair(6, 13, "muscle");
    super.addPair(7, 13, "muscle");
    super.addPair(8, 13, "muscle");
    super.addPair(9, 13, "muscle");
    super.addPair(10, 13, "muscle");
    super.addPair(11, 13, "muscle");

    // Create our ball segments
    tgStructure ball;
    /// @todo: there seems to be an issue with Muscle2P connections if the front
    /// of a super is inside the next one.
    
    for (std::size_t i = 0; i < m_segments; i++)
    {
        /// @todo: the ball is a temporary variable -- 
        /// will its destructor be called?
        /// If not, where do we delete its children?
        tgStructure* const p = new tgStructure(super);
        p->addTags(tgString("segment num", i + 1));
        ball.addChild(p); // Add a child to the ball
    }
    //conditionally compile for debugging 

#if (1)
    // Add muscles that connect the segments
    // Tag the muscles with their segment numbers so CPGs can find
    // them.
    std::vector<tgStructure*> children = ball.getChildren();
    /*
    for (std::size_t i = 1; i < children.size(); i++)
    {
        tgNodes n0 = children[i - 1]->getNodes();
        tgNodes n1 = children[i]->getNodes();
        
        ball.addPair(n0[1], n1[1],
              tgString("outer front muscle seg", i - 1) + tgString(" seg", i));
        ball.addPair(n0[2], n1[2],
              tgString("outer right muscle seg", i - 1) + tgString(" seg", i));
        ball.addPair(n0[3], n1[3],
              tgString("outer back muscle seg",  i - 1) + tgString(" seg", i));
        ball.addPair(n0[4], n1[4],
              tgString("outer top muscle seg",   i - 1) + tgString(" seg", i));
        
        ball.addPair(n0[2], n1[1],
              tgString("inner front muscle seg",  i - 1) + tgString(" seg", i));
        ball.addPair(n0[2], n1[4],
              tgString("inner right muscle seg",  i - 1) + tgString(" seg", i));
        ball.addPair(n0[3], n1[1],
              tgString("inner left muscle seg",   i - 1) + tgString(" seg", i));
        ball.addPair(n0[3], n1[4],
              tgString("inner back muscle seg",   i - 1) + tgString(" seg", i));

    }
    */
#endif

    ball.move(btVector3(0, 10, 0)); //the structure you built needs to be shifted upward, or it is stuck halfway into the ground. 
    
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    
#if (1)
    spec.addBuilder("muscle", new tgKinematicContactCableInfo(motorConfig));
#else    
    spec.addBuilder("muscle", new tgBasicContactCableInfo(motorConfig));
#endif
    
    // Create your structureInfo
    tgStructureInfo structureInfo(ball, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // Setup vectors for control
    m_allMuscles = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());
     
    m_allSegments = this->find<tgModel> ("segment");
    
#if (0)
    // Debug printing
    std::cout << "StructureInfo:" << std::endl;
    std::cout << structureInfo << std::endl;
    
    std::cout << "Model: " << std::endl;
    std::cout << *this << std::endl;
#endif
    children.clear();
    
    // Actually setup the children, notify controller
    BaseSpineModelLearning::setup(world);
}

void SuperballModelContact::teardown()
{
    
    BaseSpineModelLearning::teardown();
      
}

void SuperballModelContact::step(double dt)
{
    /* CPG update occurs in the controller so that we can decouple it
    * from the physics update
    */
    
    BaseSpineModelLearning::step(dt);  // Step any children
}
