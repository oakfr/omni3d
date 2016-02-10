#include "basic.h"
#include "camera/camera.h"

// extrinsic vector: 3 rotations, 3 translations
//Extrinsic camera 0: 0.363760 1.566527 0.364968 0.031001 0.000151 0.000043
//Extrinsic camera 1: 0.540489 1.563308 -0.718255 0.010003 -0.030317 0.000302
//Extrinsic camera 2: 1.591347 1.561258 -0.922957 -0.025783 -0.019115 0.000066
//Extrinsic camera 3: 0.294740 1.562034 2.805496 -0.025407 0.018658 -0.000115
//Extrinsic camera 4: 1.531771 1.561420 2.784971 0.010185 0.030623 -0.000295
//Extrinsic camera 5: -0.001519 -0.001678 -0.001208 0.000348 0.000319 0.032871

/* Read the unit extrinsic parameters from file
 */
void Camera::readUnitExtrinsic (const char *filename)
{
	int iCamera;
	int i,j;

	printf("Reading unit extrinsic calibration file %s...\n", filename);

	FILE *f = fopen(filename,"r");
	if (f == NULL) {
		printf("failed to open %s. No unit calibration available.\n",filename);
		assert(false);
	}

	_extrinsic.clear();
	_extrinsic_inv.clear();
	_extrinsic_dft.clear();

	for (iCamera=0;iCamera<6;iCamera++) {
		int id = -1;
		double val;

		fscanf(f,"%d",&id);
		Matd M;
		M.SetSize(4,4);
		for (i=0;i<4;i++) {
			for (j=0;j<4;j++) {
				fscanf(f,"%lf",&val);
				M[i][j] = val;
			}
		}

		assert(id == iCamera);
		Matd A = inv(M);
		_extrinsic.push_back(inv(M));
		_extrinsic_dft.push_back(inv(M));
		_extrinsic_inv.push_back(M);
	}

	fclose(f);
}

/* return the unit extrinsic param from memory
 */
Matd Camera::getUnitExtrinsic (int sensor_id)
{
	return _extrinsic[sensor_id];
}

/* Return the inverse extrinsic parameters
 */
Matd Camera::getUnitExtrinsicInv (int sensor_id)
{
	return _extrinsic_inv[sensor_id];
}

/* Return the Z-direction of a specified camera in the camera frame coordinate
 */
Vec3d Camera::getZDirection(int cameraId)
{
	Vecd X;
	X = getUnitExtrinsic(cameraId) * Vecd (4,0.0,0.0,1.0,1.0);

	return norm(Vec3d(X[0],X[1],X[2]));
}

/* Return the camera center of a sensor 
 */
Vec3d Camera::getUnitCameraCenter (int sensor_id)
{
	Matd M = getUnitExtrinsic(sensor_id);
	return Vec3d(M[0][3],M[1][3],M[2][3]);
}

/* This function computes the unit extrinsic matrices
 * from the vector returned by the Ladybug
 * we store these matrices in a file and read them with readUnitExtrinsic()
 */
void Camera::computeUnitExtrinsic(char *filename)
{
	FILE *file = fopen(filename,"w");
	if (file == NULL) {
		LOG(LEVEL_ERROR,"could not open file %s\n",filename);
		return;
	}

	Matd M,Minv;

	for (int i=0;i<6;i++) {
		computeUnitExtrinsic(i,M,Minv);
		fprintf(file,"%d\n",i);
		printMatrix(file,Minv);
		fprintf(file,"\n");
		printMatrix(file,M);
		fprintf(file,"\n");
	}

	fclose(file);

}

/* Compute the unit extrinsic params given the 6 params from the Ladybug
 */
void Camera::computeUnitExtrinsic( int sensor_id, double ext0, double ext1, double ext2, double ext3, double ext4, double ext5, Matd &M, Matd &Minv)
{

	M.SetSize(4,4);
	Minv.SetSize(4,4);

	
		Mat3d R = fromEulerTo3x3Matrix(ext0,ext1,ext2);
		Vec3d T = Vec3d(ext3,ext4,ext5);

		sub(M,3,3) = R;
		Vec3d t = -R*T;
		M[0][3] = t[0];
		M[1][3] = t[1];
		M[2][3] = t[2];
		M[3][0] = M[3][1] = M[3][2] = 0.0;
		M[3][3] = 1.0;

		sub(Minv,3,3) = inv(R);
		t = T;

		Minv[0][3] = t[0];
		Minv[1][3] = t[1];
		Minv[2][3] = t[2];
		Minv[3][0] = Minv[3][1] = Minv[3][2] = 0.0;
		Minv[3][3] = 1.0;

		printf("extrinsic ladybug [%d]\n", sensor_id);

		for (int i=0;i<4;i++) {
			for (int j=0;j<4;j++) {
				printf("%f ", Minv[i][j]);
			}
			printf("\n");
		}
}

/* this function computes the unit extrinsic matrices
 * from the vector returned by the Ladybug
 * we store these matrices in a file and read them with readUnitExtrinsic()
 */
void Camera::computeUnitExtrinsic (int sensor_id, Matd &M, Matd &Minv)
{
	M.SetSize(4,4);
	Minv.SetSize(4,4);

	if (_cameraType == LADYBUG) {
		Vecd ext;
		
		switch (sensor_id) {
		case 0:
			ext =  Vecd(6,  0.363760,  1.566527,  0.364968,  0.031001,  0.000151,  0.000043);
			//ext += Vecd(6,	-0.068000, 0.068000, -0.020000,  0.000000,  0.000000,  0.000000); // add refinement value - Aug 2005
			break;
		case 1:
			ext = Vecd(6,  0.540489,  1.563308, -0.718255,  0.010003, -0.030317,  0.000302);
			//ext += Vecd(6, -0.02400,  0.052000, 0.036000,   0.000000,  0.000000,  0.000000); // add refinement value - Aug 2005
			break;
		case 2:
			ext = Vecd(6,  1.591347,  1.561258, -0.922957, -0.025783, -0.019115,  0.000066);
			//ext += Vecd(6,	0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000); // add refinement value - Aug 2005
			break;
		case 3:
			ext = Vecd(6,  0.294740,  1.562034,  2.805496, -0.025407,  0.018658, -0.000115);
			//ext += Vecd(6,	-0.044000, 0.012000, 0.000000,  0.000000,  0.000000,  0.000000); // add refinement value - Aug 2005
			break;
		case 4:
			ext = Vecd(6,  1.531771,  1.561420,  2.784971,  0.010185,  0.030623, -0.000295);
			//ext += Vecd(6,	0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000); // add refinement value - Aug 2005
			break;
		case 5:
			ext = Vecd(6, -0.001519  -0.001678, -0.001208,  0.000348,  0.000319,  0.032871);
			//ext += Vecd(6,	-0.028000, 0.016000,  0.000000,  0.000000,  0.000000,  0.000000); // add refinement value - Aug 2005
			break;
		default:
			fprintf(stderr,"wrong sensor ID: %d\n",sensor_id);
			assert(false);
			break;
		}		

		Mat3d R = fromEulerTo3x3Matrix(ext[0],ext[1],ext[2]);
		Vec3d T = Vec3d(ext[3],ext[4],ext[5]);

		sub(M,3,3) = R;
		Vec3d t = -R*T;
		M[0][3] = t[0];
		M[1][3] = t[1];
		M[2][3] = t[2];
		M[3][0] = M[3][1] = M[3][2] = 0.0;
		M[3][3] = 1.0;

		sub(Minv,3,3) = inv(R);
		Minv[0][3] = T[0];
		Minv[1][3] = T[1];
		Minv[2][3] = T[2];
		Minv[3][0] = Minv[3][1] = Minv[3][2] = 0.0;
		Minv[3][3] = 1.0;
	}
}

/* Print out unit extrinsic parameters
 */
void Camera::printUnitExtrinsic()
{
	for (int iCamera=0;iCamera<6;iCamera++)
	{
		printf("%d\n",iCamera);
		printMatrix(inv(_extrinsic[iCamera]));
		printf("\n");
	}
}

