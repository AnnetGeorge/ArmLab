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

        try:
            self.LoadMatrices()
            self.kinectCalibrated = True
        except:
            pass

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
        world_xyz_data = [[self.ijToXyz(u,v) for u in range(640)]for v in range(480)]
        hsv_img = cv2.cvtColor(self.currentVideoFrame, cv2.COLOR_RGB2HSV)
        cubes = self.GetCubes(hsv_img, self.currentVideoFrame, world_xyz_data)
        print("cubes!!!")
        print(cubes)
        return cubes

    def detectBlocksInDepthImage(self):
        """
        TODO:
        Implement a blob detector to find blocks
        in the depth image
        """
        pass

    def getPixelColorLabel(self, u,v,hsv_img, rgb_img, world_xyz_data):
        exemplarsHSV = [[89,28,105],
                        [32,221,120],
                        [212, 52, 101],
                        [17, 208, 100],
                        [231, 143, 240],
                        [128, 0, 0],
                        [237, 152, 82],
                        [152, 64, 109],
                        [215,22,183]]
        labels=[['green'],['yellow'],['violet'],['orange'],['pink'],['black'],['red'],['blue'],['workspace']]
        world_xyz = world_xyz_data[v][u][0:3]
        # if the pixel is outside the workspace, return false
        if(world_xyz[0] > 310 or world_xyz[0] < -310 or world_xyz[1] > 310 or world_xyz[1] < -310 or world_xyz[2] > 400 or world_xyz[2] < 3):
            return ('off board', False)
        distances = [np.linalg.norm(hsv_img[v][u] - b) for b in exemplarsHSV]
        exemplarIndex = distances.index(min(distances))
        # means we found an exemplar beyond our block color exemplars
        return (labels[exemplarIndex], (exemplarIndex <= 7))

    def isPixelBlockMember(self,u,v,hsv_img, world_xyz_data, colorLabels,searchLimit = 10, searchThresh = 0.5):
        label, isBlockColorPixel = colorLabels[u][v]
        if(not isBlockColorPixel):
            return False
        else:
            numBlockColors = 0
            numValidNeighbors = 0
            for i in np.arange(-searchLimit, searchLimit, 1):
                for j in np.arange(-searchLimit, searchLimit, 1):
                    if (i == 0 and j ==0):
                        pass
                    u_neighbor = u + i
                    v_neighbor = v + j
                    if(u_neighbor > 0 and u_neighbor < 640 and v_neighbor > 0 and v_neighbor < 480):
                        numValidNeighbors += 1
                        if colorLabels[u_neighbor][v_neighbor][0] == label:
                            numBlockColors += 1
            return ((float(numBlockColors) / float(numValidNeighbors)) > searchThresh)
        
    def clusterCentroid(self, centroid, centroids):
        visitationMap = np.zeros((480,640))
        occupancyMap = np.zeros((480,640))
        targetLabel = centroid[2][0]
        centroid_U = centroid[0]
        centroid_V = centroid[1]
        clusteredCentroids = []
        queue = []
        queue.append(centroid)
        while(len(queue) > 0):
            newCentroid = queue.pop()
            newCentroidU = newCentroid[0]
            newCentroidV = newCentroid[1]
            newCentroidLabel = newCentroid[2][0]
            visitationMap[newCentroidV][newCentroidU] = 1
            if(newCentroidLabel == targetLabel):
                occupancyMap[newCentroidV][newCentroidU] = 1
                clusteredCentroids.append(newCentroid)
                for exploreU in [newCentroidU-1,newCentroidU,newCentroidU+1]:
                    for exploreV in [newCentroidV-1,newCentroidV,newCentroidV+1]:
                        if(visitationMap[exploreV][exploreU] == 0):
                            for exploreCentroid in centroids:
                                if(exploreCentroid[0] == exploreU and exploreCentroid[1] == exploreV):
                                    queue.append(exploreCentroid)
        return (clusteredCentroids, occupancyMap)

    def IsClusterCube(self, cluster, hsv_img, world_xyz_data, crossSectionBounds = [625,4225]):
        maxX = None
        maxY = None
        maxZ = None
        minX = None
        minY = None
        minZ = None
        for centroid in cluster:
            centroidXYZ = world_xyz_data[centroid[1]][centroid[0]]
            if(maxX is None or centroidXYZ[0] > maxX):
                maxX = centroidXYZ[0]
            if(maxY is None or centroidXYZ[1] > maxY):
                maxY = centroidXYZ[1]
            if(maxZ is None or centroidXYZ[2] > maxZ):
                maxZ = centroidXYZ[2]
            if(minX is None or centroidXYZ[0] < minX):
                minX = centroidXYZ[0]
            if(minY is None or centroidXYZ[1] < minY):
                minY = centroidXYZ[1]
            if(minZ is None or centroidXYZ[2] < minZ):
                minZ = centroidXYZ[2]
        deltaX = maxX - minX
        deltaY = maxY - minY
        deltaZ = maxZ - minZ
        for BBCrossSectionArea in [deltaX * deltaY,deltaX * deltaZ,deltaY*deltaZ]:
            if BBCrossSectionArea >= crossSectionBounds[0] and BBCrossSectionArea <= crossSectionBounds[1]:
                return True
        return False

    def GetClusters(self, hsv_img, rgb_img, world_xyz_data):
        colorLabels = [[self.getPixelColorLabel(u,v,hsv_img, rgb_img,world_xyz_data) for v in range(480)] for u in range(640)]
        centroids_raw = []
        for u in range(640):
            for v in range(480):
                if self.isPixelBlockMember(u,v,hsv_img,world_xyz_data,colorLabels):
                    centroids_raw.append([u,v,self.getPixelColorLabel(u,v,hsv_img,rgb_img,world_xyz_data)])
        claimedGrid = np.zeros((480,640))
        clusters = []
        for centroid in centroids_raw:
            if(claimedGrid[centroid[1]][centroid[0]] == 0):
                newCluster, clusterOccupancy = self.clusterCentroid(centroid, centroids_raw)
                clusters.append(newCluster)
                claimedGrid += clusterOccupancy
        return clusters

    def GetCubes(self,hsv_img, rgb_img, world_xyz_data, maxZSlop=3):
        clusters = self.GetClusters(hsv_img, rgb_img, world_xyz_data)
        cubeClusters = []
        cubes = []
        for cluster in clusters:
            if self.IsClusterCube(cluster, hsv_img, world_xyz_data):
                cubeClusters.append(cluster)
        for cubeCluster in cubeClusters:
            centroidXYZs = [world_xyz_data[centroid[1]][centroid[0]] for centroid in cubeCluster]
            centroidZs = [centroidXYZ[2] for centroidXYZ in centroidXYZs]
            maxZ = max(centroidZs)
            clusterXYZ = None
            nearestDist = None
            for i in range(len(centroidXYZs)):
                centroidXYZ = centroidXYZs[i]
                if((centroidXYZ[2] + maxZSlop) > maxZ):
                    zAxisDist = np.linalg.norm([centroidXYZ[0],centroidXYZ[1]])
                    if(clusterXYZ is None or zAxisDist < nearestDist):
                        clusterXYZ = centroidXYZ
                        nearestDist =  zAxisDist
                        index = i
            cubes.append((clusterXYZ, cubeCluster[i]))
        return cubes

    def LoadMatrices(self):
        self.cameraExtrinsic = np.loadtxt("./util/extrinsic_mat.cfg")
        self.depth2rgb_affine = np.loadtxt("./util/depthRGBAffine.cfg")
    
    def SaveMatrices(self):
        np.savetxt("./util/extrinsic_mat.cfg", self.cameraExtrinsic)
        np.savetxt("./util/depthRGBAffine.cfg", self.depth2rgb_affine)
