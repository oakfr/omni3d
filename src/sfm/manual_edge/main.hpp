#include "basic.h"
#include "camera.hpp"
#include "viewer/viewer.hpp"
#include "database/database.hpp"
#include "edges/edge.hpp"
#include "util/util.h"

Camera _camera;
Viewer _viewer;
Database _database;
int _sensorId;
int _frameId;
int _edgeId; // edge being picked
bool _edgeIdValid;

std::vector< std::vector<Edge2D> > _all_edges;
std::vector< std::vector<Edge2D> > _all_read_edges;
std::vector< std::vector<Edge2D> > _save_edges, _save_rectified_edges; // this is a copy of edges to optimize speed
std::vector<Edge2D> _edges;

Edge2D _manualEdge;
bool _manualEdgeBegin,_manualEdgeValid;
bool _blackScreen;
bool _drawEdges;

