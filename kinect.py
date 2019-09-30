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

        A = np.zeros((6,6))
        B = np.zeros((6,1))
        for i in range(3):
            Arow_1 = [pts1[i][0], pts1[i][1],1,0,0,0]
            Arow_2 = [0,0,0,pts1[i][0], pts1[i][1],1]
            B[i*2] = pts2[i][0]
            B[(i*2)+1] = pts2[i][1]
            A[i*2] = Arow_1
            A[(i*2)+1] = Arow_2

        # A_inv = np.linalg.inv(A)
        AtA = np.matmul(A.T, A)
        AtA_inv = np.linalg.inv(AtA)
        pseudo_invA = np.matmul(AtA_inv,A.T)
        result = np.matmul(pseudo_invA,B)
        return [[result[0][0],result[1][0],result[2][0]],[result[3][0],result[4][0],result[5][0]],[0,0,1]]
    
    def applyAffine(self, frame, affineMatrix):
        input_shape = frame.shape
        result = np.zeros(input_shape)
        for i in range(input_shape[0]):
            for j in range(input_shape[1]):
                source_mat = np.array([i,j,1]).T
                dest_mat = np.matmul(source_mat, affineMatrix)
                dest_i = int(dest_mat[0])
                dest_j = int(dest_mat[1])
                if(dest_i >= 0 and dest_i < input_shape[0] and dest_j >= 0 and dest_j < input_shape[1]):
                    result[dest_i][dest_j] = frame[i][j]
        return result

    def ijToXyz(self, i, j):
        z_pixel = self.currentDepthFrame[j][i]
        d_cal = 123.6 * np.tan(z_pixel/2842.5 + 1.1863)
        z_cal = self.extrinsicTranslation[2][0] - d_cal
        return "-", "-", z_cal

    def registerDepthFrame(self, frame):
        """
        Using an Affine transformation, transforms the depth frame to match the RGB frame
        """
        return self.applyAffine(frame, self.depth2rgb_affine)

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