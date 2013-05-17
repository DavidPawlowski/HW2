// hw2.cpp : Defines the entry point for the console application.
//



/*  -*-c++-*- 
 *  Copyright (C) 2008 Cedric Pinson <mornifle@plopbyte.net>
 *
 * This library is open source and may be redistributed and/or modified under  
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or 
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * OpenSceneGraph Public License for more details.
*/



#include "stdafx.h"
#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/Drawable>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/Array>
#include <osg/PrimitiveSet>
#include <osg/ShapeDrawable>
#include <osg/animationpath>
#include <iostream>
#include <osg/Geometry>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>
#include <osg/Material>
#include <osgAnimation/Sampler>
#include <osgAnimation/Keyframe>
#include <osgManipulator/CommandManager>
#include <osgManipulator/TabBoxDragger>
#include <osgManipulator/TabPlaneDragger>
#include <osgManipulator/TabPlaneTrackballDragger>
#include <osgManipulator/TrackballDragger>
#include <osgManipulator/Translate1DDragger>
#include <osgManipulator/Translate2DDragger>
#include <osgManipulator/TranslateAxisDragger>

#include <osgUtil/Optimizer>
#include <osg/CoordinateSystemNode>
#include <osgText/Text>

osg::ref_ptr<osg::Node> _selectedNode;


/*
//ex. 3
int _tmain(int argc, _TCHAR* argv[])
{
    // Create the root node Group.
	osg::ref_ptr<osg::Group> root = new osg::Group;
	// Read the object
	osg::ref_ptr<osg::Node> cow = osgDB::readNodeFile( "cow.osg" );
	// Define a matrix transform 
	osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
	// Set scene graph
	mt->addChild(cow);
	root->addChild(mt);

	// Create animation path
	osg::ref_ptr<osg::AnimationPath> path = new osg::AnimationPath;
	
	// Define control points
	osg::AnimationPath::ControlPoint CP0(osg::Vec3( 2.f, 1.f, 0.f ));
	osg::AnimationPath::ControlPoint CP1(osg::Vec3( -2.f, 1.f, 0.f ),osg::Quat(-osg::PI/6,osg::Vec3( 0.f, 0.f, 1.f )));
	osg::AnimationPath::ControlPoint CP2 (osg::Vec3( -2.f, -1.f, 0.f ),osg::Quat(-osg::PI*2/6,osg::Vec3( 0.f, 0.f, 1.f )),osg::Vec3( 0.5f, 0.5f, 0.5f ));
	osg::AnimationPath::ControlPoint CP3 (osg::Vec3( 2.f, -1.f, 0.f ),osg::Quat(-osg::PI*3/6,osg::Vec3( 0.f, 0.f, 1.f )));
	osg::AnimationPath::ControlPoint CP4 (osg::Vec3( 2.f, 1.f, 0.f ));

	// Insert them to the path
	path->insert( 0.0f, CP0 ); // time, point
	path->insert( 1.0f, CP1 );
	path->insert( 2.0f, CP2 );
	path->insert( 3.0f, CP3 );
	path->insert( 4.0f, CP4 );	
	path->setLoopMode(osg::AnimationPath::LoopMode::SWING);

	// Define animation path callback
	osg::ref_ptr<osg::AnimationPathCallback> APCallback = new osg::AnimationPathCallback(path.get() );
	
	// Update the matrix transform
	mt->setUpdateCallback( APCallback.get() );

	// create the view of the scene.
    osgViewer::Viewer viewer;
    viewer.setSceneData( root.get() );

    return( viewer.run() );
}
*/


//ex.1
/*

int _tmain(int argc, _TCHAR* argv[])
{
	//// The minimum code for rendering OSG in your application
	//osgViewer::Viewer viewer;
	//viewer.setSceneData( osgDB::readNodeFile( "cow.osg" ) );
	//return viewer.run();

	// Code for direct view control
	osgViewer::Viewer viewer;
	viewer.setSceneData( osgDB::readNodeFile( "cow.osgt" ) );
	viewer.getCamera()->setProjectionMatrixAsPerspective(40., 1., 1., 100. );
	// Set the background to black
	viewer.getCamera()->setClearColor( osg::Vec4( 0., 0., 0., 1. ) );
	// Create a matrix to specify a distance from the viewpoint.
	osg::Matrix trans;
	trans.makeTranslate( 0., 0., -12. );
	// Rotation angle (in radians)
	double angle( 0. );
	while (!viewer.done())
	{
		// Create the rotation matrix.
		osg::Matrix rot;
		rot.makeRotate( angle, osg::Vec3( 1., 0., 0. ) );
		angle += 0.01;
		// Set the view matrix (the concatenation of the rotation and
		// translation matrices).
		viewer.getCamera()->setViewMatrix( rot * trans );
		// Draw the next frame.
		viewer.frame();
	}
}

*/





// ex.2
/*
// Derive a class from NodeCallback to manipulate a
// MatrixTransform object's matrix.
class RotateCB : public osg::NodeCallback
{
public:
	RotateCB() : _angle( 0. ) {}
	virtual void operator()( osg::Node* node,osg::NodeVisitor* nv )
	{
		// Normally, check to make sure we have an update
		// visitor, not necessary in this simple example.
		osg::MatrixTransform* mtLeft =
		dynamic_cast<osg::MatrixTransform*>( node );
		osg::Matrix mR, mT;
		mT.makeTranslate( -6., 0., 0. );
		mR.makeRotate( _angle, osg::Vec3( 0., 0., 1. ) );
		mtLeft->setMatrix( mR * mT );
		// Increment the angle for the next from.
		_angle += 0.01;
		// Continue traversing so that OSG can process
		// any other nodes with callbacks.
		traverse( node, nv );
	}
protected:
	double _angle;
};

// Create the scene graph. This is a Group root node with two
// MatrixTransform children, which both parent a single
// Geode loaded from the cow.osg model file.
osg::ref_ptr<osg::Node>
createScene()
{
	// Load the cow model.
	osg::Node* body = osgDB::readNodeFile( "body.osgt" );
	// Data variance is STATIC because we won't modify it.
	body->setDataVariance( osg::Object::STATIC );

	// Create a MatrixTransform to display the cow on the left.
	osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
	mt->setName( "Body\nDYNAMIC" );
	// Set data variance to DYNAMIC to let OSG know that we
	// will modify this node during the update traversal.
	mt->setDataVariance( osg::Object::DYNAMIC );
	// Set the update callback.
	mt->setUpdateCallback( new RotateCB );
	osg::Matrix m;
	m.makeTranslate( -6.f, 0.f, 0.f );
	mt->setMatrix( m );
	mt->addChild( body );


	// Create the Group root node.
	osg::ref_ptr<osg::Group> root = new osg::Group;
	root->setName( "Root Node" );
	// Data variance is STATIC because we won't modify it.
	root->setDataVariance( osg::Object::STATIC );
	root->addChild( mt.get() );

	return root.get();
}

int _tmain(int argc, _TCHAR* argv[])
{
	//
	    // construct the viewer.
    osgViewer::Viewer viewer;

    std::string dragger_name = "TabBoxDragger";
    osg::Timer_t start_tick = osg::Timer::instance()->tick();


    // create a command manager
    osg::ref_ptr<osgManipulator::CommandManager> cmdMgr = new osgManipulator::CommandManager;

    
    viewer.setSceneData( createScene().get() );

    
    // pass the loaded scene graph to the viewer.
    viewer.setSceneData(addDraggerToScene(loadedModel.get(), cmdMgr.get(), dragger_name));
 
    viewer.addEventHandler(new PickModeHandler());

    return viewer.run();

	//



	// Create the viewer and set its scene data to our scene
	// graph created above.
	osgViewer::Viewer viewer;
	viewer.setSceneData( createScene().get() );
	// Set the clear color to something other than chalky blue.
	viewer.getCamera()->setClearColor(
	osg::Vec4( 1., 0., 0., 1. ) );
	// Loop and render. OSG calls RotateCB::operator()()
	// during the update traversal.
	viewer.run();
}
*/









// animation
/*
class AnimtkUpdateCallback : public osg::NodeCallback
{
public:
    META_Object(osgAnimation, AnimtkUpdateCallback);

    AnimtkUpdateCallback() 
    {
        _sampler = new osgAnimation::Vec3CubicBezierSampler;
        _playing = false;
        _lastUpdate = 0;
    }
    AnimtkUpdateCallback(const AnimtkUpdateCallback& val, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY):
        osg::Object(val, copyop),
        osg::NodeCallback(val, copyop),
        _sampler(val._sampler),
        _startTime(val._startTime),
        _currentTime(val._currentTime),
        _playing(val._playing),
        _lastUpdate(val._lastUpdate)
    {
    }

    // Callback method called by the NodeVisitor when visiting a node.
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    { 
        if (nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR && 
            nv->getFrameStamp() && 
            nv->getFrameStamp()->getFrameNumber() != _lastUpdate) {

            _lastUpdate = nv->getFrameStamp()->getFrameNumber();
            _currentTime = osg::Timer::instance()->tick();

            if (_playing && _sampler.get() && _sampler->getKeyframeContainer()) {
                osg::MatrixTransform* transform = dynamic_cast<osg::MatrixTransform*>(node);
                if (transform) {
                    osg::Vec3 result;
                    float t = osg::Timer::instance()->delta_s(_startTime, _currentTime);
                    float duration = _sampler->getEndTime() - _sampler->getStartTime();
                    t = fmod(t, duration);
                    t += _sampler->getStartTime();
                    _sampler->getValueAt(t, result);
                    transform->setMatrix(osg::Matrix::translate(result));
                }
            }
        }
        // note, callback is responsible for scenegraph traversal so
        // they must call traverse(node,nv) to ensure that the
        // scene graph subtree (and associated callbacks) are traversed.
        traverse(node,nv);
    }

    void start() { _startTime = osg::Timer::instance()->tick(); _currentTime = _startTime; _playing = true;}
    void stop() { _currentTime = _startTime; _playing = false;}

    osg::ref_ptr<osgAnimation::Vec3CubicBezierSampler> _sampler;
    osg::Timer_t _startTime;
    osg::Timer_t _currentTime;
    bool _playing;
    int _lastUpdate;
};


class AnimtkStateSetUpdateCallback : public osg::StateSet::Callback
{
public:
    META_Object(osgAnimation, AnimtkStateSetUpdateCallback);

    AnimtkStateSetUpdateCallback() 
    {
        _sampler = new osgAnimation::Vec4LinearSampler;
        _playing = false;
        _lastUpdate = 0;
    }

    AnimtkStateSetUpdateCallback(const AnimtkStateSetUpdateCallback& val, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY):
        osg::Object(val, copyop),
        osg::StateSet::Callback(val, copyop),
        _sampler(val._sampler),
        _startTime(val._startTime),
        _currentTime(val._currentTime),
        _playing(val._playing),
        _lastUpdate(val._lastUpdate)
    {
    }

    // Callback method called by the NodeVisitor when visiting a node.
    virtual void operator()(osg::StateSet* state, osg::NodeVisitor* nv)
    { 
        if (state && 
            nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR && 
            nv->getFrameStamp() && 
            nv->getFrameStamp()->getFrameNumber() != _lastUpdate) 
        {

            _lastUpdate = nv->getFrameStamp()->getFrameNumber();
            _currentTime = osg::Timer::instance()->tick();

            if (_playing && _sampler.get() && _sampler->getKeyframeContainer()) 
            {
                osg::Material* material = dynamic_cast<osg::Material*>(state->getAttribute(osg::StateAttribute::MATERIAL));
                if (material) 
                {
                    osg::Vec4 result;
                    float t = osg::Timer::instance()->delta_s(_startTime, _currentTime);
                    float duration = _sampler->getEndTime() - _sampler->getStartTime();
                    t = fmod(t, duration);
                    t += _sampler->getStartTime();
                    _sampler->getValueAt(t, result);
                    material->setDiffuse(osg::Material::FRONT_AND_BACK, result);
                }
            }
        }
    }

    void start() { _startTime = osg::Timer::instance()->tick(); _currentTime = _startTime; _playing = true;}
    void stop() { _currentTime = _startTime; _playing = false;}

    osg::ref_ptr<osgAnimation::Vec4LinearSampler> _sampler;
    osg::Timer_t _startTime;
    osg::Timer_t _currentTime;
    bool _playing;
    int _lastUpdate;
};


osg::Geode* createAxis()
{
    osg::Geode* geode  = new osg::Geode;  
    osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());

    osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());
    vertices->push_back (osg::Vec3 ( 0.0, 0.0, 0.0));
    vertices->push_back (osg::Vec3 ( 10.0, 0.0, 0.0));
    vertices->push_back (osg::Vec3 ( 0.0, 0.0, 0.0));
    vertices->push_back (osg::Vec3 ( 0.0, 10.0, 0.0));
    vertices->push_back (osg::Vec3 ( 0.0, 0.0, 0.0));
    vertices->push_back (osg::Vec3 ( 0.0, 0.0, 10.0));
    geometry->setVertexArray (vertices.get());

    osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());
    colors->push_back (osg::Vec4 (1.0f, 0.0f, 0.0f, 1.0f));
    colors->push_back (osg::Vec4 (1.0f, 0.0f, 0.0f, 1.0f));
    colors->push_back (osg::Vec4 (0.0f, 1.0f, 0.0f, 1.0f));
    colors->push_back (osg::Vec4 (0.0f, 1.0f, 0.0f, 1.0f));
    colors->push_back (osg::Vec4 (0.0f, 0.0f, 1.0f, 1.0f));
    colors->push_back (osg::Vec4 (0.0f, 0.0f, 1.0f, 1.0f));
    geometry->setColorArray (colors.get());

    geometry->setColorBinding (osg::Geometry::BIND_PER_VERTEX);    
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,6));

    geode->addDrawable( geometry.get() );
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, false);
    return geode;
}

osg::StateSet* setupStateSet()
{
    osg::StateSet* st = new osg::StateSet;
    st->setAttributeAndModes(new osg::Material, true);
    st->setMode(GL_BLEND, true);
    AnimtkStateSetUpdateCallback* callback = new AnimtkStateSetUpdateCallback;
    osgAnimation::Vec4KeyframeContainer* keys = callback->_sampler->getOrCreateKeyframeContainer();
    keys->push_back(osgAnimation::Vec4Keyframe(0, osg::Vec4(0,0,0,0)));
    keys->push_back(osgAnimation::Vec4Keyframe(2, osg::Vec4(0.5,0,0,0.5)));
    keys->push_back(osgAnimation::Vec4Keyframe(4, osg::Vec4(0,0.5,0,1)));
    keys->push_back(osgAnimation::Vec4Keyframe(6, osg::Vec4(0,0,0.5,1)));
    keys->push_back(osgAnimation::Vec4Keyframe(8, osg::Vec4(1,1,1,0.5)));
    keys->push_back(osgAnimation::Vec4Keyframe(10, osg::Vec4(0,0,0,0)));
    callback->start();
    st->setUpdateCallback(callback);
    return st;
}

osg::Node* setupCube()
{
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f,0.0f,0.0f),2)));
    geode->setStateSet(setupStateSet());
    return geode;
}

osg::MatrixTransform* setupAnimtkNode()
{
	// These are the positions through which the object will pass
    osg::Vec3 v[5];
    v[0] = osg::Vec3(0,0,0);
    v[1] = osg::Vec3(10,-50,0);
    v[2] = osg::Vec3(30,-10,20);
    v[3] = osg::Vec3(-10,20,-20);
    v[4] = osg::Vec3(0,0,0);
	// The transformation
    osg::MatrixTransform* node = new osg::MatrixTransform;
	// Callback
    AnimtkUpdateCallback* callback = new AnimtkUpdateCallback;
	// Keyframes
    osgAnimation::Vec3CubicBezierKeyframeContainer* keys = callback->_sampler->getOrCreateKeyframeContainer();
    keys->push_back(osgAnimation::Vec3CubicBezierKeyframe(0, osgAnimation::Vec3CubicBezier(
                                                        v[0], // pos
                                                        v[0] + (v[0] - v[3]), // p1 
                                                        v[1] - (v[1] - v[0]) // p2
                                                        )));
    keys->push_back(osgAnimation::Vec3CubicBezierKeyframe(2, osgAnimation::Vec3CubicBezier(
                                                        v[1], // pos
                                                        v[1] + (v[1] - v[0]),
                                                        v[2] - (v[2] - v[1])
                                                        )));
    keys->push_back(osgAnimation::Vec3CubicBezierKeyframe(4, osgAnimation::Vec3CubicBezier(
                                                        v[2], // pos
                                                        v[2] + (v[2] - v[1]),
                                                        v[3] - (v[3] - v[2])
                                                        )));
    keys->push_back(osgAnimation::Vec3CubicBezierKeyframe(6, osgAnimation::Vec3CubicBezier(
                                                        v[3], // pos
                                                        v[3] + (v[3] - v[2]),
                                                        v[4] - (v[4] - v[3])
                                                        )));
    keys->push_back(osgAnimation::Vec3CubicBezierKeyframe(8, osgAnimation::Vec3CubicBezier(
                                                        v[4], // pos
                                                        v[4] + (v[4] - v[3]),
                                                        v[0] - (v[0] - v[4])
                                                        )));

    callback->start();
    node->setUpdateCallback(callback);
    node->addChild(setupCube());
    return node;
}

int _tmain(int argc, _TCHAR* argv[])
//int main (int argc, char* argv[])
{
    //osg::ArgumentParser arguments(&argc, argv);
    osgViewer::Viewer viewer;//(arguments);

    osgGA::TrackballManipulator* manipulator = new osgGA::TrackballManipulator();
    viewer.setCameraManipulator(manipulator);
  
	// Without animation
    osg::Group* root = new osg::Group;
    root->setInitialBound(osg::BoundingSphere(osg::Vec3(10,0,10), 30));
    root->addChild(createAxis());

	// with animation
    osg::MatrixTransform* node = setupAnimtkNode();
    node->addChild(createAxis());
    root->addChild(node);

    viewer.setSceneData( root );
    viewer.realize();

    while (!viewer.done()) 
    {
        viewer.frame();
    }

}

*/





//animation related

osg::Geode* createAxis()
{
    osg::Geode* geode  = new osg::Geode;  
    osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());

    osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());
    vertices->push_back (osg::Vec3 ( 0.0, 0.0, 0.0));
    vertices->push_back (osg::Vec3 ( 10.0, 0.0, 0.0));
    vertices->push_back (osg::Vec3 ( 0.0, 0.0, 0.0));
    vertices->push_back (osg::Vec3 ( 0.0, 10.0, 0.0));
    vertices->push_back (osg::Vec3 ( 0.0, 0.0, 0.0));
    vertices->push_back (osg::Vec3 ( 0.0, 0.0, 10.0));
    geometry->setVertexArray (vertices.get());

    osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());
    colors->push_back (osg::Vec4 (1.0f, 0.0f, 0.0f, 1.0f));
    colors->push_back (osg::Vec4 (1.0f, 0.0f, 0.0f, 1.0f));
    colors->push_back (osg::Vec4 (0.0f, 1.0f, 0.0f, 1.0f));
    colors->push_back (osg::Vec4 (0.0f, 1.0f, 0.0f, 1.0f));
    colors->push_back (osg::Vec4 (0.0f, 0.0f, 1.0f, 1.0f));
    colors->push_back (osg::Vec4 (0.0f, 0.0f, 1.0f, 1.0f));
    geometry->setColorArray (colors.get());

    geometry->setColorBinding (osg::Geometry::BIND_PER_VERTEX);    
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,6));

    geode->addDrawable( geometry.get() );
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, false);
    return geode;
}

class AnimtkUpdateCallback : public osg::NodeCallback
{
public:
    META_Object(osgAnimation, AnimtkUpdateCallback);

    AnimtkUpdateCallback() 
    {
        _sampler = new osgAnimation::Vec3CubicBezierSampler;
        _playing = false;
        _lastUpdate = 0;
    }
    AnimtkUpdateCallback(const AnimtkUpdateCallback& val, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY):
        osg::Object(val, copyop),
        osg::NodeCallback(val, copyop),
        _sampler(val._sampler),
        _startTime(val._startTime),
        _currentTime(val._currentTime),
        _playing(val._playing),
        _lastUpdate(val._lastUpdate)
    {
    }

    // Callback method called by the NodeVisitor when visiting a node.
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    { 
        if (nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR && 
            nv->getFrameStamp() && 
            nv->getFrameStamp()->getFrameNumber() != _lastUpdate) {

            _lastUpdate = nv->getFrameStamp()->getFrameNumber();
            _currentTime = osg::Timer::instance()->tick();

            if (_playing && _sampler.get() && _sampler->getKeyframeContainer()) {
                osg::MatrixTransform* transform = dynamic_cast<osg::MatrixTransform*>(node);
                if (transform) {
                    osg::Vec3 result;
                    float t = osg::Timer::instance()->delta_s(_startTime, _currentTime);
                    float duration = _sampler->getEndTime() - _sampler->getStartTime();
                    t = fmod(t, duration);
                    t += _sampler->getStartTime();
                    _sampler->getValueAt(t, result);
                    transform->setMatrix(osg::Matrix::translate(result));
                }
            }
        }
        // note, callback is responsible for scenegraph traversal so
        // they must call traverse(node,nv) to ensure that the
        // scene graph subtree (and associated callbacks) are traversed.
        traverse(node,nv);
    }

    void start() { _startTime = osg::Timer::instance()->tick(); _currentTime = _startTime; _playing = true;}
    void stop() { _currentTime = _startTime; _playing = false;}

    osg::ref_ptr<osgAnimation::Vec3CubicBezierSampler> _sampler;
    osg::Timer_t _startTime;
    osg::Timer_t _currentTime;
    bool _playing;
    int _lastUpdate;
};


osg::MatrixTransform* setupAnimtkNode()
{
	// These are the positions through which the object will pass
    osg::Vec3 v[5];
    v[0] = osg::Vec3(0,0,0);
    v[1] = osg::Vec3(10,-50,0);
    v[2] = osg::Vec3(30,-10,20);
    v[3] = osg::Vec3(-10,20,-20);
    v[4] = osg::Vec3(0,0,0);
	// The transformation
    osg::MatrixTransform* node = new osg::MatrixTransform;
	// Callback
    AnimtkUpdateCallback* callback = new AnimtkUpdateCallback;
	// Keyframes
    osgAnimation::Vec3CubicBezierKeyframeContainer* keys = callback->_sampler->getOrCreateKeyframeContainer();
    keys->push_back(osgAnimation::Vec3CubicBezierKeyframe(0, osgAnimation::Vec3CubicBezier(
                                                        v[0], // pos
                                                        v[0] + (v[0] - v[3]), // p1 
                                                        v[1] - (v[1] - v[0]) // p2
                                                        )));
    keys->push_back(osgAnimation::Vec3CubicBezierKeyframe(2, osgAnimation::Vec3CubicBezier(
                                                        v[1], // pos
                                                        v[1] + (v[1] - v[0]),
                                                        v[2] - (v[2] - v[1])
                                                        )));
    keys->push_back(osgAnimation::Vec3CubicBezierKeyframe(4, osgAnimation::Vec3CubicBezier(
                                                        v[2], // pos
                                                        v[2] + (v[2] - v[1]),
                                                        v[3] - (v[3] - v[2])
                                                        )));
    keys->push_back(osgAnimation::Vec3CubicBezierKeyframe(6, osgAnimation::Vec3CubicBezier(
                                                        v[3], // pos
                                                        v[3] + (v[3] - v[2]),
                                                        v[4] - (v[4] - v[3])
                                                        )));
    keys->push_back(osgAnimation::Vec3CubicBezierKeyframe(8, osgAnimation::Vec3CubicBezier(
                                                        v[4], // pos
                                                        v[4] + (v[4] - v[3]),
                                                        v[0] - (v[0] - v[4])
                                                        )));

    callback->start();
    node->setUpdateCallback(callback);
    //node->addChild(setupCube());
    return node;
}


// manipulator
osgManipulator::Dragger* createDragger(const std::string& name)
{
    osgManipulator::Dragger* dragger = 0;
    if ("TabPlaneDragger" == name)
    {
        osgManipulator::TabPlaneDragger* d = new osgManipulator::TabPlaneDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else if ("TabPlaneTrackballDragger" == name)
    {
        osgManipulator::TabPlaneTrackballDragger* d = new osgManipulator::TabPlaneTrackballDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else if ("TrackballDragger" == name)
    {
        osgManipulator::TrackballDragger* d = new osgManipulator::TrackballDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else if ("Translate1DDragger" == name)
    {
        osgManipulator::Translate1DDragger* d = new osgManipulator::Translate1DDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else if ("Translate2DDragger" == name)
    {
        osgManipulator::Translate2DDragger* d = new osgManipulator::Translate2DDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else if ("TranslateAxisDragger" == name)
    {
        osgManipulator::TranslateAxisDragger* d = new osgManipulator::TranslateAxisDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else
    {
        osgManipulator::TabBoxDragger* d = new osgManipulator::TabBoxDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }

    
 
    return dragger;
}


osg::Node* createHUD()
{
    osg::Geode* geode = new osg::Geode();
    
    std::string timesFont("fonts/arial.ttf");

    osg::StateSet* stateset = geode->getOrCreateStateSet();
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    osgText::Text* text = new  osgText::Text;
    geode->addDrawable( text );

    osg::Vec3 position(50.0f,50.0f,0.0f);
    text->setPosition(position);
    text->setText("Use the Tab key to switch between the trackball and pick modes.");
    text->setFont(timesFont);

    osg::Camera* camera = new osg::Camera;

    // set the projection matrix
    camera->setProjectionMatrix(osg::Matrix::ortho2D(0,1280,0,1024));

    // set the view matrix    
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrix(osg::Matrix::identity());

    // only clear the depth buffer
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);

    // draw subgraph after main camera view.
    camera->setRenderOrder(osg::Camera::POST_RENDER);

    camera->addChild(geode);
    
    return camera;
}

osg::Node* addDraggerToScene(osg::Node* scene, osgManipulator::CommandManager* cmdMgr, const std::string& name)
{
    scene->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

    osgManipulator::Selection* selection = new osgManipulator::Selection;
    selection->addChild(scene);

    osgManipulator::Dragger* dragger = createDragger(name);

    osg::Group* root = new osg::Group;
    root->addChild(dragger);
    root->addChild(selection);
    root->addChild(createHUD());

    float scale = scene->getBound().radius() * 1.6;
    dragger->setMatrix(osg::Matrix::scale(scale, scale, scale) *
                       osg::Matrix::translate(scene->getBound().center()));
    cmdMgr->connect(*dragger, *selection);

    return root;
}


class PickModeHandler : public osgGA::GUIEventHandler
{
    public:
        enum Modes
        {
            VIEW = 0,
            PICK
        };

        PickModeHandler():
            _mode(VIEW), 
            _activeDragger(0)
        {
        }        
        
        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa,
                    osg::Object*, osg::NodeVisitor*)
        {
            osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
            if (!view) return false;

            if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Tab &&
                ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN &&
                _activeDragger == 0)
            {
                _mode = ! _mode;
            }
            
            if (VIEW == _mode) return false;

            switch (ea.getEventType())
            {
                case osgGA::GUIEventAdapter::PUSH:
                {
                    osgUtil::LineSegmentIntersector::Intersections intersections;

                    _pointer.reset();

                    if (view->computeIntersections(ea.getX(),ea.getY(),intersections))
                    {
                        _pointer.setCamera(view->getCamera());
                        _pointer.setMousePosition(ea.getX(), ea.getY());

                        for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
                            hitr != intersections.end();
                            ++hitr)
                        {
                            _pointer.addIntersection(hitr->nodePath, hitr->getLocalIntersectPoint());
                        }
                        for (osg::NodePath::iterator itr = _pointer._hitList.front().first.begin();
                             itr != _pointer._hitList.front().first.end();
                             ++itr)
                        {
                            osgManipulator::Dragger* dragger = dynamic_cast<osgManipulator::Dragger*>(*itr);
                            if (dragger)
                            {

                                dragger->handle(_pointer, ea, aa);
                                _activeDragger = dragger;
                                break;
                            }                   
                        }
                    }
                }
                case osgGA::GUIEventAdapter::DRAG:
                case osgGA::GUIEventAdapter::RELEASE:
                {
                    if (_activeDragger)
                    {
                        _pointer._hitIter = _pointer._hitList.begin();
                        _pointer.setCamera(view->getCamera());
                        _pointer.setMousePosition(ea.getX(), ea.getY());

                        _activeDragger->handle(_pointer, ea, aa);
                    }
                    break;
                }
        default:
            break;
            }

            if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE)
            {
                _activeDragger = 0;
                _pointer.reset();
            }

            return true;
        }
        
    private:
        unsigned int _mode;
        osgManipulator::Dragger* _activeDragger;
        osgManipulator::PointerInfo _pointer;
};


// hw1 addition

osg::Node* createScene(osgManipulator::CommandManager* cmdMgr)
{
	// Load the models

	osg::Node* body = osgDB::readNodeFile( "body.osgt" );
	
	osg::ref_ptr<osg::MatrixTransform> transform_1 = new osg::MatrixTransform;

	body->setName("body");
	osg::Group* root = new osg::Group;

	//osg::ref_ptr<osg::Group> root = new osg::Group;

	osg::ref_ptr<osg::Material> matirial = new osg::Material;
    matirial->setColorMode(osg::Material::DIFFUSE);
    matirial->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1));
    matirial->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(1, 1, 1, 1));
    matirial->setShininess(osg::Material::FRONT_AND_BACK, 64.0f);
    root->getOrCreateStateSet()->setAttributeAndModes(matirial.get(), osg::StateAttribute::ON);


	//transform_1->addChild(addDraggerToScene( body ,cmdMgr,"TabBoxDragger"));
	//root->addChild(addDraggerToScene(transform_1.get(), cmdMgr,"TabBoxDragger"));
	transform_1.get()->addChild(addDraggerToScene(body,cmdMgr,"TabBoxDragger"));
	//
	root->addChild(transform_1.get());
	return root;

}
// End hw1 addition



int _tmain(int argc, _TCHAR* argv[])
{


    // construct the viewer.
    osgViewer::Viewer viewer;

    std::string dragger_name = "TabBoxDragger";
    osg::Timer_t start_tick = osg::Timer::instance()->tick();


    // create a command manager
    osg::ref_ptr<osgManipulator::CommandManager> cmdMgr = new osgManipulator::CommandManager;

    // if no model has been successfully loaded report failure.
    bool tragger2Scene(true);
    osg::ref_ptr<osg::Node> loadedModel = createScene(cmdMgr.get());
    tragger2Scene=false;
    

    osg::Timer_t end_tick = osg::Timer::instance()->tick();
    std::cout << "Time to load = "<<osg::Timer::instance()->delta_s(start_tick,end_tick)<<std::endl;


    // optimize the scene graph, remove redundant nodes and state etc.
    osgUtil::Optimizer optimizer;
    optimizer.optimize(loadedModel.get());

	osg::MatrixTransform* node = setupAnimtkNode();
    // pass the loaded scene graph to the viewer.
    if ( tragger2Scene ) {
        viewer.setSceneData(addDraggerToScene(loadedModel.get(), cmdMgr.get(), dragger_name));
    } else { 
        viewer.setSceneData(loadedModel.get());
    }
    viewer.addEventHandler(new PickModeHandler());

    return viewer.run();
}