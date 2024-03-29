/**
 * @file RoboCupCtrl.h
 *
 * This file declares the class RoboCupCtrl.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include <QList>
#include <list>

#include "SimRobotCore2.h"
#include "Oracle.h"
#include "GameController.h"

class ControllerRobot;

/**
 * The class implements the SimRobot controller for RoboCup.
 */
class RoboCupCtrl : public SimRobot::Module, public SimRobotCore2::CollisionCallback {
public:
  static RoboCupCtrl* controller;            /**< A pointer to the SimRobot controller. */
  static SimRobot::Application* application; /**< The interface to the SimRobot GUI */
  GameController gameController;

  /**
   * Constructor.
   */
  RoboCupCtrl(SimRobot::Application& application);

  /**
   * Adds a scene graph object to the scene graph displayed in SimRobot
   * @param object The scene graph object to add
   * @param parent The parent scene graph object (e.g. a category or null)
   * @param flags Some flags for registering the scene graph object (see SimRobot::Flag)
   */
  void addView(SimRobot::Object* object, const SimRobot::Object* parent = 0, int flags = 0);

  /**
   * Adds a scene graph object to the scene graph displayed in SimRobot
   * @param object The scene graph object to add
   * @param categoryName The full name of the parent categroy
   * @param flags Some flags for registering the scene graph object (see SimRobot::Flag)
   */
  void addView(SimRobot::Object* object, const QString& categoryName, int flags = 0);

  /**
   * Adds a category to the scene graph displayed in SimRobot that can be used for grouping views
   * @param name The name of the category
   * @param parent The parent scene graph object (e.g. a category or null)
   * @param icon The icon used to list the category in the scene graph
   * @return The category
   */
  SimRobot::Object* addCategory(const QString& name, const SimRobot::Object* parent = 0, const char* icon = 0);

  /**
   * Adds a category to the scene graph displayed in SimRobot that can be used for grouping views
   * @param name The name of the category
   * @param parentName The name of the parent scene graph object (e.g. a category)
   * @return The category
   */
  SimRobot::Object* addCategory(const QString& name, const QString& parentName);

  /**
   * Removes a category from the scene graph displayed in SimRobot
   * @param name The name of the category
   * @return Whether anything was deleted from the scene graph
   */
  bool removeCategory(const QString& name);

  /**
   * The function returns the full name of the robot currently constructed.
   * @return The name of the robot.
   */
  static const char* getRobotFullName() { return controller->robotName; }

  /**
   * The function returns the name of the robot associated to the current thread.
   * @return The name of the robot.
   */
  std::string getRobotName() const;

  /**
   * The function returns the model of the robot associated to the current thread.
   * @return The model of the robot.
   */
  std::string getModelName() const;

protected:
  const char* robotName;              /**< The name of the robot currently constructed. */
  std::list<ControllerRobot*> robots; /**< The list of all robots. */
  int simStepLength;                  /**< The length of one simulation step (in ms) */
  bool dragTime;                      /**< Drag simulation to avoid running faster then realtime. */
  int time;                           /**< The simulation time. */
  unsigned lastTime;                  /**< The last time execute was called. */
  std::string statusText;             /**< The text to be printed in the status bar. */

  /**
   * Destructor.
   */
  virtual ~RoboCupCtrl();

  /**
   * The function is called to initialize the module.
   */
  virtual bool compile();

  /**
   * The function is called in each simulation step.
   */
  virtual void update();

  /**
   * The callback function.
   * Called whenever the geometry at which this interface is registered collides with another geometry.
   * @param geom1 The geometry at which the interface has been registered
   * @param geom2 The other geometry
   */
  virtual void collided(SimRobotCore2::Geometry& geom1, SimRobotCore2::Geometry& geom2);

  /**
   * Has to be called by derived class to start processes.
   */
  void start();

  /**
   * Has to be called by derived class to stop processes.
   */
  void stop();

private:
  QList<SimRobot::Object*> views; /**< List of registered views */
};
