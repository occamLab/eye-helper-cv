import numpy as np
import numpy.linalg
import scipy
import epipolar as ep
import cv2

def find_nullspace(M):
    """

    Convenient function that finds the nullspace of a matrix M using SVD.

    Inputs:
    M -> matrix

    Outputs:
    null_space -> array of nullspace vectors

    """

    U, S, V = np.linalg.svd(M)
    eps = 1e-13 #that threshold
    null_mask = (S <= eps)
    null_space = scipy.compress(null_mask, V, axis=0)

    return null_space

def find_projective_transform(F, T, img1path, img2path):
    """
    img1path - i
    img2path - iprime

    Implementing image rectification strategy mentioned in "Computing Rectifying Homographies for Stereo Vision."
    """

    img1 = cv2.imread(img1path,0)  #queryimage # left image
    img2 = cv2.imread(img2path,0) #trainimage # right image
    width, height = img1.shape
    widthprime, heightprime = img2.shape

    # e is in the nullspace of F
    # F*e = 0
    # F.T * eprime = 0
    e = find_nullspace(F)[0]
    eprime = find_nullspace(F.T)[0]

    # print 'e', e
    # print 'eprime', eprime

    # P * Ptranspose = that jazz
    # and prime friends
    P_PT = float(width)*height/12.0 * np.array([width**2 - 1, 0, 0, 
                                                0, height**2 - 1, 0, 
                                                0, 0, 0]).reshape(3,3)
    Pprime_PprimeT = float(widthprime)*heightprime/12.0 * np.array([widthprime**2 - 1, 0, 0, 
                                                                    0, heightprime**2 - 1, 0, 
                                                                    0, 0, 0]).reshape(3,3)

    # Pc * Pctranspose = that jazz 
    # and prime friends
    Pc_PcT = 1.0/4 * np.array([ (width-1)**2, (width-1)*(height-1), 2*(width-1), 
                                (width-1)*(height-1), (height-1)**2, 2*(height-1), 
                                2*(width-1), 2*(height-1), 4 ]).reshape(3,3)
    Pprimec_PprimecT = 1.0/4 * np.array([ (widthprime-1)**2, (widthprime-1)*(heightprime-1), 2*(widthprime-1), 
                                          (widthprime-1)*(heightprime-1), (heightprime-1)**2, 2*(heightprime-1), 
                                          2*(widthprime-1), 2*(heightprime-1), 4 ]).reshape(3,3)

    # [e]x
    e_cross = np.array([0, -e[2], e[1], e[2], 0, -e[0], -e[1], e[0], 0]).reshape(3, 3)

    #might want to put these in a function (and call it twice, for prime and not prime things)
    # calculating A and B and D and key and prime friends

    # applying the A = [e]x.T * P_PT * [e]x expression in the paper
    A = np.dot(np.dot(e_cross.T, P_PT), e_cross)
    Aprime = np.dot(np.dot(F.T,Pprime_PprimeT), F)  

    # however... numpy is complaining that A isn't positive definite. (but it is, we looked at the eigenvalues)
    # because of either numerical rounding errors (threshold in numpy for cholesky to work) 
    # or because of a previous error in our code...
    # workaround: use eigenvalue decomposition; change the really small eigenvalue slightly so numpy does not regard it as zero
    # and then cholesky decomposition works 
    Aeigvals, Aeigvecs = np.linalg.eig(A)
    Aeigvals[2] *= 10 
    A =  np.dot(np.dot(Aeigvecs, np.diag(Aeigvals)), Aeigvecs.T)

    Aprimeeigvals, Aprimeeigvecs = np.linalg.eig(Aprime)
    Aprimeeigvals[2] *= 10 
    Aprime =  np.dot(np.dot(Aprimeeigvecs, np.diag(Aprimeeigvals)), Aprimeeigvecs.T)

    B = np.dot(np.dot(e_cross.T, Pc_PcT), e_cross)
    Bprime = np.dot(np.dot(F.T,Pprimec_PprimecT), F) 
    
    D = np.linalg.cholesky(A)
    Dprime = np.linalg.cholesky(Aprime)

    # is there a numpy equivalent of MATLAB's backslash?
    Dinv = np.linalg.inv(D) 
    Dinvprime = np.linalg.inv(Dprime)

    # key is the matrix we take the eigenthings ; the goal is to find the largest eigenvalue
    key = np.dot(np.dot(Dinv.T,B), Dinv) 
    keyprime = np.dot(np.dot(Dinvprime.T,Bprime), Dinvprime) 

    eigvals, eigvecs = np.linalg.eig(key)
    eigvalsprime, eigvecsprime = np.linalg.eig(keyprime)

    #find largest eigenvalue
    largest_eigval = eigvals[0]
    for i in range(len(eigvals)):
        if abs(eigvals[i]) > largest_eigval:
            largest_eigval = eigvals[i]
            y = eigvecs[i] #corresponding eigenvector to that largest eigenvalue

    largest_eigvalprime = eigvalsprime[0]
    for i in range(len(eigvalsprime)):
        if abs(eigvalsprime[i]) > largest_eigvalprime:
            largest_eigvalprime = eigvalsprime[i]
            yprime = eigvecs[i] #corresponding eigenvector to that largest eigenvalue

    # at long last! z
    z = np.dot(Dinv, y)
    zprime = np.dot(Dinvprime, yprime)
    z = (z/np.linalg.norm(z) + zprime/np.linalg.norm(zprime))/2

    w = np.dot(e_cross, z)
    wprime = np.dot(F, z)

    Hp = np.array([1, 0, 0,
                   0, 1, 0,
                   w[0], w[1], 1]).reshape(3,3)
    Hpprime = np.array([1, 0, 0,
                        0, 1, 0,
                        wprime[0], wprime[1], 1]).reshape(3,3)

    return w, wprime, Hp, Hpprime

if __name__ == '__main__':
    mtx, dst, rvecs, tvecs = ep.calibrate_from_chessboard()
    R,T, F, pts1, pts2 = ep.extract_r_t_f('img_0.jpg', 'img_1.jpg', mtx, dst)
    w, wprime, Hp, Hpprime = find_projective_transform(F, T, 'img_0.jpg', 'img_1.jpg')    

    # center = np.array([0.0, 0.0, 0.0]).reshape(1,3)
    # new_center = (center+T).dot(R)
    # print 'R: ', R 
    # print 'T: ', T
    # # print 'first center: ', center
    # # print 'new center: ', new_center

    # pts1 = np.array(pts1)
    # pts2 = np.array(pts2)
