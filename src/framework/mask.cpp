#include "main.h"

// compute the edge mask corresponding to a a set of 3D lines
// due to distortion, one line may give rise to several masks
// 
void MyGlWindow::computeMask (edgeVector lines, int edge_step)
{	
	_camera->clearMasks();
	int i,j;
	
	// compute mask from reprojected lines
	for (i=0;i<lines.size();i++)
		computeMask(lines[i],edge_step);
	
	// add mask for existing chained edges
	for (int cameraId = 0; cameraId < 6; cameraId++) {
		
		for (i=0;i<_camera->_frame._edgeplanes_chained[cameraId].size();i++) {
			EdgePlane edgeplane = _camera->_frame._edgeplanes_chained[cameraId][i];
			
			if (edgeplane._cameraId != cameraId)
				continue;
			
			for (j=0;j<edgeplane._edgeIds.size();j++) {
				
				Edge2D edge = _camera->_frame._edges[cameraId][edgeplane._edgeIds[j]];
				
				_camera->AddMask(cameraId,NULL,edge,false);
			}
		}
	}
}

// compute the edge mask corresponding to a given 3D line
// due to distortion, one line will rise to several submasks
// 
void MyGlWindow::computeMask (Edge *line, int edge_step)
{
	if ( line == NULL )
		return;
	
	intVector cameras;
	_camera->findSensors( line, cameras );
	
	for (int i=0; i < cameras.size(); i++) {
		int img = cameras[i];
		
		edge2DVector edges;
		_camera->projectAndSubdivide( img, line->getA(), line->getB(), edge_step, edges);
		
		for (edge2DVector::iterator iter = edges.begin(); iter != edges.end(); iter++ ) {
			iter->_lineId = line->_id;
			iter->_type = REPROJECTED;
			_camera->AddMask(img,line,*iter,false);
		}
	}
	
}

// compute mask from edgeplane
// the camera pose is reset because the edgeplane is expressed in the camera coordinate frame
void MyGlWindow::computeMask ( EdgePlane edgeplane, int edge_step )
{
	ExtrinsicParameters pose = _camera->getPose();
	_camera->resetPose();
	
	int img = edgeplane._cameraId;
	
	double radius = 50.0;
	
	edge2DVector edges;
	_camera->projectAndSubdivide( img, radius*edgeplane._a, radius*edgeplane._b, edge_step, edges);
	
	for (edge2DVector::iterator iter = edges.begin(); iter != edges.end(); iter++ ) {
		iter->_type = REPROJECTED;
		_camera->AddMask(img,NULL,*iter,false);
	}
	
	_camera->setPose( pose );
}

// project a set of 3D lines onto the camera and store the image edges in the camera structure
//
void MyGlWindow::reprojectEdges( edgeVector &lines )
{
	int edge_step = 10; // edge step in pixels
	
	// clear the reprojected edges
	_camera->clearReprojectedEdges();
	
	for (int i=0;i<lines.size();i++) {
		Edge *line = lines[i];
		
		intVector cameras;
		_camera->findSensors( line, cameras );
		
		// for each camera seeing the line, reproject on the camera
		for (int i=0;i<cameras.size();i++) {
			
			int cameraId = cameras[i];

			edge2DVector edges;
			_camera->projectAndSubdivide( cameraId, line->getA(), line->getB(), edge_step, edges);
			
			int color_id = -1;

			switch ( line->status ) {
				case ACCEPTED:
					color_id = 0;
					break;
				case PENDING:
					color_id = 1;
					break;
				case UNKNOWN:
					color_id = 2;
					break;
				default:
					color_id = -1;
					break;
			}

			for (int j=0;j<edges.size();j++)  {
				edges[j]._id = color_id;
				_camera->_frame._edges_reprojected[cameraId].push_back( edges[j] ); //  and save in the camera structure
			}
		}
	}
}

// project a set of 3D lines onto the camera and store the image edges in the camera structure
// warning: may have to take a lock here in multi-threaded mode
//
void MyGlWindow::reprojectEdges( edgeVector &lines, ExtrinsicParameters &pose )
{
	ExtrinsicParameters save_pose = _camera->getPose();

	_camera->setPose( pose );
	
	reprojectEdges( lines );

	_camera->setPose( save_pose );
}

// compute mask from edgeplane and line
// the mask is computed from the edgeplane, but extended using the reprojection of the line
// the camera pose is reset because the edgeplane is expressed in the camera coordinate frame
void MyGlWindow::computeMask ( EdgePlane edgeplane, Edge *line, int edge_step )
{
	ExtrinsicParameters pose = _camera->getPose();
	_camera->resetPose();
	
	double radius = 50.0;
	
	int img = edgeplane._cameraId;
	
	Edge2D edge_image, edge_line;
	
	// compute the image edge
	if ( !_camera->project( img, radius*edgeplane._a, radius*edgeplane._b, edge_image ) ) {
		_camera->setPose( pose );
		return;
	}
	
	// if possible, compute the projected edge and extend the image edge with it
	if ( line != NULL ) {
		
		_camera->setPose( pose );
		
		if ( !_camera->project( img, line->getA(), line->getB(), edge_line ) ) {
			return;
		}
		
		edge_image.extend( edge_line );
		
		_camera->resetPose();
	}
	
	// subdivide the image edge
	// subdivide the edge
	Intrinsic intr = _camera->getIntrinsic(img);
	int n = (double)edge_image.length() / ((double)edge_step/intr.getWidth()) + 1;
	
	Vec2d p = edge_image._a;
	Vec2d q = edge_image._b;
	Vec2d u = ( q - p ) / n;
	
	for (int i=0; i < n; i++ ) {
		Vec2d p1 = _camera->distort( img, p + i * u );
		Vec2d p2 = _camera->distort( img, p + (i + 1) * u );
		
		if ( intr.visible( p1 ) && intr.visible( p2 ) ) {
			Edge2D ledge = Edge2D( p1, p2 );
			ledge._type = REPROJECTED;
			
			_camera->AddMask(img,NULL,ledge, false);
		}
	}
	
	/*Intrinsic intr = _camera->getIntrinsic(img);
	int n = edge_image.length() / edge_step + 1;
	Vec2d p = intr.rectify(intr.normalize( edge_image._a ));
	Vec2d q = intr.rectify(intr.normalize( edge_image._b ));
	Vec2d u = ( q - p ) / n;
	
	  for (int i=0; i < n; i++ ) {
	  Vec2d p1 = _camera->distort( img, p + i * u );
	  Vec2d p2 = _camera->distort( img, p + (i + 1) * u );
	  Edge2D ledge = Edge2D( p1, p2 );
	  ledge._type = REPROJECTED;
	  
		_camera->AddMask(img,NULL,ledge, true);
		}
	*/
	
	_camera->setPose( pose );
}
