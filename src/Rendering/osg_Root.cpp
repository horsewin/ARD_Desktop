/*
 * osg_Root.cpp
 *
 *  Created on: 2012/12/20
 *      Author: umakatsu
 */

#include "src\Rendering\osg_Root.h"
#include "src\Rendering\osg_Object.h"
#include "src\Rendering\osg_Init.h"

extern const int NUMBER_CAR;
extern const float MIN_HAND_PIX;
extern const int MAX_NUM_HANDS;
vector<int> objVectorDeletable;
vector<int> fingersIdx;

osg_Root::osg_Root()
{
	mAddModelAnimation	= 0.0;
	mOsgArInputButton	= -1;
	mOsgArAddModelIndex	= -1;

	//Create Height field
	CreateGround();
}

osg_Root::~osg_Root()
{
	fingerIndex.clear();
	osg_uninit();
}

/** create quad at specified position. */
osg::Drawable* osg_Root::createSquare(const osg::Vec3& corner,const osg::Vec3& width,const osg::Vec3& height, osg::Image* image)
{
    // set up the Geometry.
    osg::Geometry* geom = new osg::Geometry;

    osg::Vec3Array* coords = new osg::Vec3Array(4);
    (*coords)[0] = corner;
    (*coords)[1] = corner+width;
    (*coords)[2] = corner+width+height;
    (*coords)[3] = corner+height;


    geom->setVertexArray(coords);

    osg::Vec3Array* norms = new osg::Vec3Array(1);
    (*norms)[0] = width^height;
    (*norms)[0].normalize();
    
    geom->setNormalArray(norms);
    geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

    osg::Vec2Array* tcoords = new osg::Vec2Array(4);
    (*tcoords)[0].set(0.0f,0.0f);
    (*tcoords)[1].set(1.0f,0.0f);
    (*tcoords)[2].set(1.0f,1.0f);
    (*tcoords)[3].set(0.0f,1.0f);
    geom->setTexCoordArray(0,tcoords);
    
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));
    
    if (image)
    {
        osg::StateSet* stateset = new osg::StateSet;
        osg::Texture2D* texture = new osg::Texture2D;
        texture->setImage(image);
        stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
        geom->setStateSet(stateset);
    }
    
    return geom;
}

osg::Drawable* osg_Root::createAxis(const osg::Vec3& corner,const osg::Vec3& xdir,const osg::Vec3& ydir,const osg::Vec3& zdir)
{
    // set up the Geometry.
    osg::Geometry* geom = new osg::Geometry;

    osg::Vec3Array* coords = new osg::Vec3Array(6);
    (*coords)[0] = corner;
    (*coords)[1] = corner+xdir;
    (*coords)[2] = corner;
    (*coords)[3] = corner+ydir;
    (*coords)[4] = corner;
    (*coords)[5] = corner+zdir;

    geom->setVertexArray(coords);

    osg::Vec4 x_color(1.0f,.0f,.0f,1.0f);
    osg::Vec4 y_color(0.0f,1.0f,.0f,1.0f);
    osg::Vec4 z_color(.0f,0.0f,1.0f,1.0f);

    osg::Vec4Array* color = new osg::Vec4Array(6);
    (*color)[0] = x_color;
    (*color)[1] = x_color;
    (*color)[2] = y_color;
    (*color)[3] = y_color;
    (*color)[4] = z_color;
    (*color)[5] = z_color;
    
    geom->setColorArray(color);
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,6));
    
    osg::StateSet* stateset = new osg::StateSet;
    osg::LineWidth* linewidth = new osg::LineWidth();
    linewidth->setWidth(14.0f);
    stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    geom->setStateSet(stateset);
    
    return geom;
}

osg::Node* osg_Root::createMilestone()
{
    // create the root node which will hold the model.
    osg::Group* milestone = new osg::Group();

    // add the drawable into a single goede to be shared...
    osg::Billboard* center = new osg::Billboard();
    center->setMode(osg::Billboard::POINT_ROT_EYE);
    center->addDrawable(
        createSquare(osg::Vec3(-0.5f,0.0f,-0.5f),osg::Vec3(1.0f,0.0f,0.0f),osg::Vec3(0.0f,0.0f,1.0f),osgDB::readImageFile("Images/reflect.rgb")),
        osg::Vec3(0.0f,0.0f,0.0f));
        
    osg::Geode* axis = new osg::Geode();
    axis->addDrawable(createAxis(osg::Vec3(0.0f,0.0f,0.0f),osg::Vec3(150.0f,0.0f,0.0f),osg::Vec3(0.0f,150.0f,0.0f),osg::Vec3(0.0f,0.0f,150.0f)));


    milestone->addChild(center);
    milestone->addChild(axis);

    return milestone;
}

void osg_Root::osg_init(double *projMatrix) 
{
	mVideoImage = new osg::Image();
	mGLImage = cvCreateImage(cvSize(512,512), IPL_DEPTH_8U, 3);

	mViewer.addEventHandler(new osgViewer::WindowSizeHandler());
	mViewer.setUpViewInWindow(100, 100, WINDOW_WIDTH, WINDOW_HEIGHT);
	
	mViewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
	mViewer.setKeyEventSetsDone(0);

	osg::ref_ptr<osg::Group> root = new osg::Group();
	mViewer.setSceneData(root);

	// ----------------------------------------------------------------
	// Video background
	// ----------------------------------------------------------------
	osg::ref_ptr<osg::Camera> bgCamera = new osg::Camera();
	bgCamera->getOrCreateStateSet()->setRenderBinDetails(0, "RenderBin");
	bgCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	bgCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
	bgCamera->getOrCreateStateSet()->setMode(GL_LIGHTING, GL_TRUE);
	bgCamera->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, GL_FALSE);
	bgCamera->setProjectionMatrixAsOrtho2D(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT);
	
	osg::ref_ptr<osg::Geometry> geom = osg::createTexturedQuadGeometry(osg::Vec3(0, 0, 0), osg::X_AXIS * WINDOW_WIDTH, osg::Y_AXIS * WINDOW_HEIGHT, 0, 1, 1, 0);
	geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, new osg::Texture2D(mVideoImage));
	
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	geode->addDrawable(geom.get());
	bgCamera->addChild(geode.get());
	root->addChild(bgCamera.get());
	
	// ----------------------------------------------------------------
	// Foreground 3D content
	// ----------------------------------------------------------------
	osg::ref_ptr<osg::Camera> fgCamera = new osg::Camera();
	fgCamera->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
	fgCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT);
	fgCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
	fgCamera->setProjectionMatrix(osg::Matrix(projMatrix));
	root->addChild(fgCamera.get());

	arTrackedNode = new ARTrackedNode();
	fgCamera->addChild(arTrackedNode);
};

void osg_Root::osg_uninit() 
{
	arTrackedNode->stop();
	cvReleaseImage(&mGLImage);
}

void osg_Root::osg_inittracker(string markerName, int maxLengthSize, int maxLengthScale) 
{
	static bool hasInit = false;

	if (hasInit) {
		arTrackedNode->removeChildren(0,arTrackedNode->getNumChildren());
		celicaIndex = arTrackedNode->addMarkerContent(markerName, maxLengthSize, maxLengthScale, mShadowedScene);
		arTrackedNode->setVisible(celicaIndex, true);
		return;
	}

	arTrackedNode->removeChildren(0,arTrackedNode->getNumChildren());

	//Create rendering world and set parameters of this world----->
	// Set shadow node
	//osg::ref_ptr<osgShadow::ShadowTexture> sm = new osgShadow::ShadowTexture; //V
	//osg::ref_ptr<osgShadow::MyShadowMap> sm = new osgShadow::MyShadowMap; //Adrian
	osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap; //Atsushi
	//sm->setLight(arTrackedNode->getLight()); //Atsushi
	sm->setTextureSize( osg::Vec2s(1024, 1024) ); //Adrian
	sm->setTextureUnit( 1 );

	mShadowedScene = new osgShadow::ShadowedScene;
	mShadowedScene->setShadowTechnique( sm.get() );
	mShadowedScene->setReceivesShadowTraversalMask( rcvShadowMask );
	mShadowedScene->setCastsShadowTraversalMask( castShadowMask );
	mShadowedScene->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON); //Adrian  

	//set param of lighting
	osg::ref_ptr<osg::LightSource> source = new osg::LightSource; //V
	source->getLight()->setPosition( osg::Vec4(4.0, 8.0, 10.0, 0.0) ); //V
	source->getLight()->setAmbient( osg::Vec4(0.1, 0.1, 0.1, 1.0) ); //V
	source->getLight()->setDiffuse( osg::Vec4(0.9, 0.9, 0.9, 1.0) ); //V
	mShadowedScene->addChild(source);

	//<-----

#if CAR_SIMULATION == 1
	mOsgInit->CreateCarUnit(mOsgObject);

	//Add these cars to rendering scene
	mShadowedScene->addChild( mOsgObject->car_transform.at(0) );
	mShadowedScene->addChild( mOsgObject->car_transform.at(1) );

	for(int i = 0 ; i < 2; i++)  { 
		for(int j = 0 ; j < 4; j++)  { 
			mShadowedScene->addChild(mOsgObject->wheel_transform[i].at(j));
		}
	} 
#endif /* CAR_SIMULATION == 1 */

	celicaIndex = arTrackedNode->addMarkerContent(markerName, maxLengthSize, maxLengthScale, mShadowedScene);
	arTrackedNode->setVisible(celicaIndex, true);

/*Adrian */
	{
		// Create the Heightfield Geometry
		mOsgInit->SetHeightfield(mOsgObject);

		// Create the containing geode
		osg::ref_ptr< osg::Geode > geode = new osg::Geode(); 
		geode->addDrawable(mOsgObject->mHeightFieldGeometry_quad);
		geode->addDrawable(mOsgObject->mHeightFieldGeometry_line);

		//Create the containing transform
		float scale = 10; float x = 0; float y = 0; float z = 0;
		osg::ref_ptr< osg::PositionAttitudeTransform > mt = new osg::PositionAttitudeTransform();
		mt->setScale(osg::Vec3d(scale,scale,scale));
		mt->setAttitude(osg::Quat(0,0,0,1));       
		mt->setPosition(osg::Vec3d(x, y, z)); 
		mt->addChild( geode.get() );

		//Set up the depth testing for the landscale
		osg::Depth * depth = new osg::Depth();
		depth->setWriteMask(true); 
		depth->setFunction(osg::Depth::Function::LEQUAL);
		mt->getOrCreateStateSet()->setAttributeAndModes(depth, osg::StateAttribute::ON);

		//Set up the shadow masks
		mt->setNodeMask( rcvShadowMask );
		mt->getOrCreateStateSet()->setRenderBinDetails(0, "RenderBin");
		mShadowedScene->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");

		//At the heightmap twice, once for shadowing and once for occlusion
		arTrackedNode->addModel(mt);
		mShadowedScene->addChild(mt);

#if CAR_SIMULATION == 1
		mOsgInit->SetCarRenderBin(mOsgObject);
#endif

	}/*Adrian*/

	hasInit = true;

	/* DEBUG code */
	//for confirmation about the direction of axis
	//osg::Node * axis = createMilestone();
	//axis->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");
	//mShadowedScene->addChild(axis);

	//Create OSG Menu
#if USE_OSGMENU==1
	osgMenu = new ARMM::osg_Menu();
	OsgInitMenu();
	OsgInitModelMenu();
#endif
}

void osg_Root::osg_render(IplImage *newFrame, osg::Quat *q,osg::Vec3d  *v, osg::Quat wq[][4], osg::Vec3d wv[][4], CvMat *cParams, CvMat *cDistort, std::vector <osg::Quat> q_array, std::vector<osg::Vec3d>  v_array) 
{
	if(mAddModelAnimation >0.001 || mAddModelAnimation < -0.001) //means "!= 0.0"
	{
		ModelButtonAnimation();
	}

	cvResize(newFrame, mGLImage);
	cvCvtColor(mGLImage, mGLImage, CV_BGR2RGB);
	mVideoImage->setImage(mGLImage->width, mGLImage->height, 0, 3, GL_RGB, GL_UNSIGNED_BYTE, (unsigned char*)mGLImage->imageData, osg::Image::NO_DELETE);

#ifdef USE_ARMM_SERVER_VIEW
#if CAR_SIMULATION == 1
	if(car_transform.at(0) && car_transform.at(1)) {
		for(int i = 0; i < 2; i++) {
			car_transform.at(i)->setAttitude(q[i]);
			car_transform.at(i)->setPosition(v[i]);
			for(int j = 0; j < 4; j++) {
				wheel_transform[i].at(j)->setAttitude(wq[i][j]);
				wheel_transform[i].at(j)->setPosition(wv[i][j]);
			}
		}
	}
#endif /* CAR_SIMULATION == 1 */

	//update the position of each virtual object
	for(unsigned int i = 0; i < q_array.size(); i++) 
	{
		obj_transform_array.at(i)->setAttitude(q_array.at(i));
		obj_transform_array.at(i)->setPosition(v_array.at(i));
		osg::Vec3 pos = obj_transform_array.at(i)->getPosition();
		//printf("%d,%s:%.2f  %.2f  %.2f \n", i, obj_node_array.at(i)->getName().c_str(), v_array.at(i).x(), v_array.at(i).y(), v_array.at(i).z());
		//printf("POS=(%.2f, %.2f, %.2f)\n",pos.x(), pos.y(), pos.z());
	}

	if (!mViewer.done()) 
	{
		if (CAPTURE_SIZE.width != REGISTRATION_SIZE.width || CAPTURE_SIZE.height != REGISTRATION_SIZE.height) 
		{
			double scale = double(REGISTRATION_SIZE.width)/double(CAPTURE_SIZE.width);
			IplImage *scaledImage = cvCreateImage(cvSize(newFrame->width*scale, newFrame->height*scale), newFrame->depth, newFrame->nChannels); cvResize(newFrame, scaledImage);
			arTrackedNode->processFrame(scaledImage, cParams, cDistort);
			cvReleaseImage(&scaledImage);
		} 
		else 
		{
			arTrackedNode->processFrame(newFrame, cParams, cDistort);
		}
		mViewer.frame();
	}
#endif

	//change the condition of regular state if lists of models is shown in AR space into invisible condition
	if( !mAddArModel && IsModelButtonVisibiilty())
	{
		ToggleModelButtonVisibility();
	}
}

void osg_Root::setOSGTrimeshScale(float scale)
{
	TrimeshScale = scale;
}

void osg_Root::ShowGroundGeometry()
{
	mOsgObject->mHeightFieldGeometry_line->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::OFF);	
	mOsgObject->mGroundQuadColor->pop_back();
	mOsgObject->mGroundQuadColor->push_back(osg::Vec4(1,1,1,0.2));
	mOsgObject->mGroundLineColor->pop_back();
	mOsgObject->mGroundLineColor->push_back(osg::Vec4(1,0.2,0,1.0));
}

void osg_Root::HideGroundGeometry()
{
	mOsgObject->mHeightFieldGeometry_line->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);	
	mOsgObject->mGroundQuadColor->pop_back();
	mOsgObject->mGroundQuadColor->push_back(osg::Vec4(1,1,1,0.0));
	mOsgObject->mGroundLineColor->pop_back();
	mOsgObject->mGroundLineColor->push_back(osg::Vec4(1,1,1,0.0));
}

void osg_Root::osg_UpdateHeightfieldTrimesh(float *ground_grid)
{
	int index =0;
	for(int i = 0; i < NUM_VERTS_X-1; i++) {
		for(int j = 0; j < NUM_VERTS_Y-1; j++) {
			float x = (float)(i- center_trimesh.x)*TrimeshScale; 
			float y = (float)(j- (120-center_trimesh.y))*TrimeshScale;
			mOsgObject->mHeightFieldPoints->at(index++) = osg::Vec3(x, y, ground_grid[j*NUM_VERTS_X+i]); 
			mOsgObject->mHeightFieldPoints->at(index++) = osg::Vec3(x+TrimeshScale, y, ground_grid[j*NUM_VERTS_X+i+1]);
			mOsgObject->mHeightFieldPoints->at(index++) = osg::Vec3(x+TrimeshScale, y+TrimeshScale, ground_grid[(j+1)*NUM_VERTS_X+i+1]); 
			mOsgObject->mHeightFieldPoints->at(index++) = osg::Vec3(x, y+TrimeshScale, ground_grid[(j+1)*NUM_VERTS_X+i]);
		}
	}
	mOsgObject->mHeightFieldGeometry_quad->dirtyDisplayList();
	mOsgObject->mHeightFieldGeometry_line->dirtyDisplayList();
}

void osg_Root::osgAddObjectNode(osg::Node* n) 
{
	int index = obj_node_array.size();

	obj_node_array.push_back(n);
	obj_transform_array.push_back(new osg::PositionAttitudeTransform());
	obj_transform_array.at(index)->setAttitude(osg::Quat(
		osg::DegreesToRadians(50.f), osg::Vec3d(1.0, 0.0, 0.0),
		osg::DegreesToRadians(-50.f), osg::Vec3d(0.0, 1.0, 0.0),
		osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 0.0, -1.0)
		));
	obj_transform_array.at(index)->setPosition(osg::Vec3d(0.0, 0.0, 0.0));
	obj_transform_array.at(index)->setNodeMask(castShadowMask);
	obj_transform_array.at(index)->addChild(obj_node_array.at(index));

	mShadowedScene->addChild( obj_transform_array.at(index) );
	obj_transform_array.at(index)->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
	mShadowedScene->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");

	printf("Object number: %d added \n", index);
	std::cout << obj_transform_array.at(index)->getAttitude() << endl;
}

void osg_Root::osg_createHand(int index, float x, float y, float world_scale, float ratio) 
{
//	float sphere_size = 0.5;
	float sphere_size = world_scale * ratio;
	cout << "Sphere size = " << world_scale*ratio << endl;
	//float sphere_size = world_scale; // test

	printf("Hand%d Created : ws=%f, ratio=%f\n", index, world_scale, ratio);
	hand_object_global_array.push_back(new osg::PositionAttitudeTransform());
	for(int i = 0; i < MIN_HAND_PIX; i++) 
	{
		for(int j = 0; j < MIN_HAND_PIX; j++) 
		{
			//create a part of hands 
			osg::ref_ptr< osg::Sphere > sphere = new osg::Sphere(osg::Vec3d(0,0,0), sphere_size);
		    osg::ref_ptr< osg::ShapeDrawable> shape = new osg::ShapeDrawable( sphere.get() );
			shape->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			shape->setColor(osg::Vec4(1, 1, 0, 1));
			osg::ref_ptr< osg::Geode> geode = new osg::Geode();
			hand_object_shape_array.push_back(shape.get());
			geode->addDrawable( shape.get() );

			hand_object_transform_array[index].push_back(new osg::PositionAttitudeTransform());
			int curr = hand_object_transform_array[index].size()-1;
			hand_object_transform_array[index].at(curr)->setScale(osg::Vec3d(SPHERE_SCALE,SPHERE_SCALE,SPHERE_SCALE));
			hand_object_transform_array[index].at(curr)->setPosition(osg::Vec3d(j*SPHERE_SCALE, i*SPHERE_SCALE, 1000));
			hand_object_transform_array[index].at(curr)->addChild( geode.get() );
			hand_object_transform_array[index].at(curr)->setNodeMask(rcvShadowMask);
			hand_object_global_array.at(index)->addChild( hand_object_transform_array[index].at(curr));
		}
	}

	hand_object_global_array.at(index)->setPosition(osg::Vec3d(0,0,0));
	mShadowedScene->addChild( hand_object_global_array.at(index) );

	//set rendering order
	hand_object_global_array.at(index)->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");
	mShadowedScene->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");
}

//void osg_UpdateHand(int index, float x, float y, float *grid) {
void osg_Root::osg_UpdateHand(int index, float *x, float *y, float *grid) 
{
	const int DEPTH_SCALE = 10; // to convert cm to mm scale

	if(!VectorBoundChecker(hand_object_global_array, index)) return;

	//ëñç∏ï˚å¸ÇÕKinect viewÇ©ÇÁÇ›ÇΩÇ∆Ç´ÇÃtop-leftÇ©ÇÁtop-rightÇ…å¸Ç©Ç§
	for(int i = 0; i < MIN_HAND_PIX; i++) 
	{
		//ëñç∏ï˚å¸ÇÕKinect viewÇ©ÇÁÇ›ÇΩÇ∆Ç´ÇÃbottom-leftÇ©ÇÁtop-leftÇ…å¸Ç©Ç§
		for(int j = 0; j < MIN_HAND_PIX; j++) 
		{
			int curr = i*MIN_HAND_PIX+j;
			//osg::Vec3d pos = hand_object_transform_array[index].at(curr)->getPosition();
			if(grid[curr] > 0 && grid[curr]<100 )
			{
				hand_object_transform_array[index].at(curr)->setPosition(osg::Vec3d(x[curr]*SPHERE_SCALE, y[curr]*SPHERE_SCALE, grid[curr]*DEPTH_SCALE));

				//printf("Pos, sensor %d = %f, %f, %f\n", curr, x[curr], y[curr], grid[curr]);
				hand_object_shape_array[curr]->setColor(osg::Vec4(0.9451, 0.7333, 0.5765, 1));
			}
			else
			{
				hand_object_transform_array[index].at(curr)->setPosition(osg::Vec3d(x[curr]*SPHERE_SCALE, y[curr]*SPHERE_SCALE, -5000*DEPTH_SCALE));
				//hand_object_transform_array[index].at(curr)->setPosition(osg::Vec3d(pos.x(), pos.y(), 100*SPHERE_SCALE));
//				if(voxel[index] < 0)
//					world_sphere_transform_array.at(index)->setStateSet(
//				else
//					s->setAllChildrenOn();
			//if(curr%10 == 0)
			//	printf("Pos, sensor %d = %f, %f, %f\n", curr, x[curr], y[curr], grid[curr]);
			}
		}
	}

	//assign fingertips
	fingersIdx.clear();
	REP(fingerTips,fingerIndex.size())
	{
		int idx =	fingerIndex.at(fingerTips);

		//if(grid[idx] > 0 && grid[idx]<100){}
		//else
		{
			bool fitting = false;
			int shift = 1;
			do{
				int tmpIdx;
				for(int i=-shift; i<=shift; i++)
				{
					for(int j=-shift; j<=shift; j++)
					{
						tmpIdx = idx + (i*MIN_HAND_PIX+j);
						if(tmpIdx < 0 || tmpIdx >= HAND_GRID_SIZE) continue;
						if(grid[tmpIdx] > 0 && grid[tmpIdx]<100)
						{
							fingersIdx.push_back(tmpIdx);
							//fitting = true;
							hand_object_shape_array[tmpIdx]->setColor(osg::Vec4(1, 0, 0, 1));
						}
					}
					//if(fitting) break;
				}
				shift++;
			//}while(!fitting || shift<=2);
			}while(shift<=2);
		}
	}

}

void osg_Root::OsgInitMenu()
{
	osgMenu->CreateMenuPane();

	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > pTransArray = 
		osgMenu->getObjMenuTransformArray();

	//add menu object into the AR scene
	osg::ref_ptr<osg::Group> menu = new osg::Group;
	REP(idx, pTransArray.size())
	{
		pTransArray.at(idx)->setNodeMask(castShadowMask);
		menu->addChild(pTransArray.at(idx));
	}

	osg::ref_ptr<osg::PositionAttitudeTransform> menuTrans = new osg::PositionAttitudeTransform;
	const osg::Vec3d BASEPOSITION(0,0,0);
	const osg::Quat BASEATTITUDE = osg::Quat(
			osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 0.0, 1.0)
	);

	menuTrans->setAttitude(BASEATTITUDE);
	menuTrans->setPosition(BASEPOSITION);
	menuTrans->addChild(menu.get());

	mShadowedScene->addChild(menuTrans.get());

	osgMenu->setObjMenuTransformArray(pTransArray);
}

void osg_Root::OsgInitModelMenu()
{
	osgMenu->CreateModelButtonCloud();

	std::vector< osg::ref_ptr<osg::PositionAttitudeTransform> > pTransArray = osgMenu->getMenuModelTransArray();
	//add menu object into the AR scene
	osg::ref_ptr<osg::Group> menu = new osg::Group;
	REP(idx, pTransArray.size())
	{
		pTransArray.at(idx)->setNodeMask(castShadowMask);
		menu->addChild(pTransArray.at(idx));
	}

	osg::ref_ptr<osg::PositionAttitudeTransform> menuTrans = new osg::PositionAttitudeTransform;
	const osg::Vec3d BASEPOSITION(0,0,0);
	const osg::Quat BASEATTITUDE = osg::Quat(
			osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 0.0, 1.0)
	);
	menuTrans->setAttitude(BASEATTITUDE);
	menuTrans->setPosition(BASEPOSITION);
	menuTrans->addChild(menu.get());

	mShadowedScene->addChild(menuTrans.get());

	osgMenu->setMenuModelTransArray(pTransArray);
}

void osg_Root::ModelButtonInput()
{
	//rendering list of models
	//rendering new button for cancal action
	ToggleModelButtonVisibility();
	mAddArModel = true;

	//disappearing all buttons and virtual objects temporary 
	ToggleMenuVisibility();
	ToggleVirtualObjVisibility();

	//add menu object into the AR scene

	//play some effect
	PlaySound(_T("machine_call.wav"), NULL, SND_ASYNC);	
}

void osg_Root::ToggleMenuVisibility()
{
	std::vector< osg::ref_ptr<osg::PositionAttitudeTransform> > pMenuTransArray  = osgMenu->getObjMenuTransformArray();

	double shiftVal = 4;
	REP(idx, pMenuTransArray.size())
	{
		if(pMenuTransArray.at(idx)->getNodeMask() == castShadowMask)
		{
			pMenuTransArray.at(idx)->setNodeMask(invisibleMask);

			//appear ar model buttons
			mAddModelAnimation = shiftVal;
		}
		else
		{
			pMenuTransArray.at(idx)->setNodeMask(castShadowMask);

			//disappear ar model buttons
			//mAddModelAnimation = -1*shiftVal;
		}
	}

	osgMenu->setObjMenuTransformArray(pMenuTransArray);
	osgMenu->ToggleMenuButtonState();
}

void osg_Root::ToggleModelButtonVisibility()
{
	std::vector< osg::ref_ptr<osg::PositionAttitudeTransform> > pModelTransArray = osgMenu->getMenuModelTransArray();

	if(pModelTransArray.empty()) return;

	unsigned int nodeMask = pModelTransArray.at(0)->getNodeMask() == castShadowMask?
		invisibleMask : castShadowMask;

	REP(idx, pModelTransArray.size())
	{
		pModelTransArray.at(idx)->setNodeMask(nodeMask);
	}

	osgMenu->setMenuModelTransArray(pModelTransArray);
	osgMenu->ToggleModelButtonState();
}

void osg_Root::ToggleVirtualObjVisibility()
{
	REP(i,obj_transform_array.size())
	{
		unsigned int mask = obj_transform_array.at(i)->getNodeMask() ^ (castShadowMask);
		obj_transform_array.at(i)->setNodeMask(mask);
	}
}

bool osg_Root::IsMenuVisibiilty()
{
	return osgMenu->isMenuButtonState();
}

bool osg_Root::IsModelButtonVisibiilty()
{
	return osgMenu->isModelButtonState();
}

void osg_Root::ModelButtonAnimation()
{
	std::vector< osg::ref_ptr<osg::PositionAttitudeTransform> > pModelTransArray = osgMenu->getMenuModelTransArray();

	//check if valid models are found?
	if(pModelTransArray.empty())
	{
		cerr << "No model button is found in osg.h" << endl;
		return;
	}

	//set the action in current frame
	double posZ = pModelTransArray.at(0)->getPosition().z();
	const double zThreshold = 6.0;
	if(posZ > zThreshold)
	{
		mAddModelAnimation = 0;
		return;
	}

	//set the pos of each model in current frame
	REP(idx, pModelTransArray.size())
	{
		osg::Vec3 newPos = pModelTransArray.at(idx)->getPosition();
		newPos.set(newPos.x(), newPos.y(), newPos.z() + mAddModelAnimation);
		pModelTransArray.at(idx)->setPosition(newPos);		
	}

}

void osg_Root::ResetModelButtonPos()
{
	std::vector< osg::ref_ptr<osg::PositionAttitudeTransform> > pModelTransArray = osgMenu->getMenuModelTransArray();

	if(pModelTransArray.empty())
	{
		cerr << "No model button is found in osg.h" << endl;
		return;
	}

	REP(idx, pModelTransArray.size())
	{
		osg::Vec3 newPos = pModelTransArray.at(idx)->getPosition();
		newPos.set(newPos.x(), newPos.y(), osgMenu->GetInitPosZ());
		pModelTransArray.at(idx)->setPosition(newPos);		
	}

}

void osg_Root::CreateGround()
{
	mOsgInit->CreateHeightfield(mOsgObject);
}