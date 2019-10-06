import cv2
import numpy as np
from PyQt4.QtGui import QImage
import freenect

class Kinect():
    def __init__(self):
        self.currentVideoFrame = np.array([])
        self.currentDepthFrame = np.array([])
        self.cameraIntrinsic = self.loadCameraCalibration()
        self.extrinsicTranslation = None
        self.extrinsicRotation = None
        self.extrinsicRotation_matrix = None
        self.cameraExtrinsic = None
        self.focalMM = 2.9
        if(freenect.sync_get_depth() == None):
            self.kinectConnected = False
        else:
            self.kinectConnected = True
        
        # mouse clicks & calibration variables
        self.depth2rgb_affine = np.float32([[1,0,0],[0,1,0]])
        self.kinectCalibrated = False
        self.last_click = np.array([0,0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5,2),int)
        self.depth_click_points = np.zeros((5,2),int)

        """ Extra arrays for colormaping the depth image"""
        self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthCM=np.array([])

        """ block info """
        self.block_contours = np.array([])

    def captureVideoFrame(self):
        """                      
        Capture frame from Kinect, format is 24bit RGB    
        """
        if(self.kinectConnected):
            self.currentVideoFrame = freenect.sync_get_video()[0]
        else:
            self.loadVideoFrame()
        self.processVideoFrame()
        

    def processVideoFrame(self):
        cv2.drawContours(self.currentVideoFrame,self.block_contours,-1,(255,0,255),3)


    def captureDepthFrame(self):
        """                      
        Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        if(self.kinectConnected):
            if(self.kinectCalibrated):
                self.currentDepthFrame = self.registerDepthFrame(freenect.sync_get_depth()[0])
            else:
                self.currentDepthFrame = freenect.sync_get_depth()[0]
        else:
            self.loadDepthFrame()

    
    def loadVideoFrame(self):
        self.currentVideoFrame = cv2.cvtColor(
            cv2.imread("data/ex0_bgr.png",cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB
            )

    def loadDepthFrame(self):
        self.currentDepthFrame = cv2.imread("data/ex0_depth16.png",0)

    def convertFrame(self):
        """ Converts frame to format suitable for Qt  """
        try:
            img = QImage(self.currentVideoFrame,
                             self.currentVideoFrame.shape[1],
                             self.currentVideoFrame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def convertDepthFrame(self):
        """ Converts frame to a colormaped format suitable for Qt  
            Note: this cycles the spectrum over the lowest 8 bits
        """
        try:

            """ 
            Convert Depth frame to rudimentary colormap
            """
            self.DepthHSV[...,0] = self.currentDepthFrame
            self.DepthHSV[...,1] = 0x9F
            self.DepthHSV[...,2] = 0xFF
            self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)
            cv2.drawContours(self.DepthCM,self.block_contours,-1,(0,0,0),3)

            img = QImage(self.DepthCM,
                             self.DepthCM.shape[1],
                             self.DepthCM.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def getAffineTransform(self, coord1, coord2):
        """
        Given 2 sets of corresponding coordinates, 
        find the affine matrix transform between them.

        TODO: Rewrite this function to take in an arbitrary number of coordinates and 
        find the transform without using cv2 functions
        """
        pts1 = coord1[::].astype(np.float32)
        pts2 = coord2[::].astype(np.float32)
        numPoints = len(pts1)
        A = np.zeros((numPoints*2,6))
        B = np.zeros((numPoints*2,1))
        for i in range(numPoints):
            Arow_1 = [pts1[i][0], pts1[i][1],1,0,0,0]
            Arow_2 = [0,0,0,pts1[i][0], pts1[i][1],1]
            B[i*2] = pts2[i][0]
            B[(i*2)+1] = pts2[i][1]
            A[i*2] = Arow_1
            A[(i*2)+1] = Arow_2

        AtA = np.matmul(A.T, A)
        AtA_inv = np.linalg.inv(AtA)
        pseudo_invA = np.matmul(AtA_inv,A.T)
        result = np.matmul(pseudo_invA,B)
        return [[result[0][0],result[1][0],result[2][0]],[result[3][0],result[4][0],result[5][0]]]
    
    def applyAffine(self, frame, affineMatrix):
        input_shape = frame.shape
        result = np.zeros(input_shape, dtype=np.uint64)
        for input_y in range(input_shape[0]):
            for input_x in range(input_shape[1]):
                source_mat = np.array([float(input_x),float(input_y),1.0]).T
                dest_mat = np.matmul(affineMatrix,source_mat)
                dest_x = int(dest_mat[0])
                dest_y = int(dest_mat[1])
                if(dest_x >= 0 and dest_x < input_shape[1] and dest_y >= 0 and dest_y < input_shape[0]):
                    result[dest_y][dest_x] = frame[input_y][input_x]
        return result
    
    def applyAffineCV(self, frame, affineMatrix):
        result = cv2.warpAffine(np.array(frame),np.array(affineMatrix),(frame.shape[1], frame.shape[0]))
        return result

    def imgXyToCamXYZ(self, imgX, imgY):
        u0 = self.cameraIntrinsic[0][2]
        v0 = self.cameraIntrinsic[1][2]
        alpha = self.cameraIntrinsic[0][0]
        beta = self.cameraIntrinsic[1][1]
        z_pixel = self.currentDepthFrame[imgY][imgX]
        Zc = 123.6 * np.tan(z_pixel/2842.5 + 1.1863)
        Xc = ((imgX - u0) * Zc)/alpha
        Yc = ((imgY - v0) * Zc)/beta
        return np.array([Xc,Yc,Zc])

    def ijToXyz(self, input_x, input_y):
        camCoords = self.imgXyToCamXYZ(input_x, input_y)
        camCoordsHomogeneous = np.array(np.append(camCoords, 1.0)).T
        worldCoords = np.matmul(np.linalg.inv(self.cameraExtrinsic),camCoordsHomogeneous)
        return worldCoords

    def registerDepthFrame(self, frame):
        """
        Using an Affine transformation, transforms the depth frame to match the RGB frame
        """
        return self.applyAffineCV(frame, self.depth2rgb_affine)

    def registerExtrinsicMatrix(self, rVec, tVec):
        self.extrinsicTranslation = tVec
        self.extrinsicRotation = rVec
        self.extrinsicRotation_matrix = cv2.Rodrigues(rVec)[0]
        self.cameraExtrinsic = np.zeros((4,4))
        self.cameraExtrinsic[0][3] = tVec[0]
        self.cameraExtrinsic[1][3] = tVec[1]
        self.cameraExtrinsic[2][3] = tVec[2]
        self.cameraExtrinsic[3][3] = 1.0
        for i in [0,1,2]:
            for j in [0,1,2]:
                self.cameraExtrinsic[i][j] = self.extrinsicRotation_matrix[i][j]
        print("extrinsic mat")
        print(self.cameraExtrinsic)

    def loadCameraCalibration(self):
        """
        Load camera intrinsic matrix from file
        """
        return np.loadtxt("./util/intrinsic_mat.cfg")
    
    def blockDetector(self):
        """
        TODO:
        Implement your block detector here.  
        You will need to locate
        blocks in 3D space
        """
        pass

    def detectBlocksInDepthImage(self):
        """
        TODO:
        Implement a blob detector to find blocks
        in the depth image
        """
        pass