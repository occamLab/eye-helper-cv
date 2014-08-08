import numpy as np
import numpy.linalg

def find_projective_transform(F, e, img1path, img2path):
    """
    img1path - i
    img2path - iprime

    Implementing image rectification strategy mentioned in "Computing Rectifying Homographies for Stereo Vision."
    """
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


    #might want to put these in a function (and call it twice, for prime and not prime things)
    # calculating A and B and D and key and prime friends
    A = np.dot(np.dot(np.dot(eT, P_PT), e))
    Aprime = np.dot(np.dot(F.T,Pprime_PprimeT), F) 

    B = np.dot(np.dot(np.dot(eT, Pc_PcT), e))
    Bprime = np.dot(np.dot(F.T,Pprimec_PprimecT), F) 

    D = np.linalg.cholesky(A)
    Dprime = np.linalg.cholesky(Aprime)

    Dinv = np.linalg.inv(D)
    Dinvprime = np.linalg.inv(Dprime)

    key = np.dot(np.dot(Dinv.T,B), Dinv) 
    keyprime = np.dot(np.dot(Dinvprime.T,Bprime), Dinvprime) 

    eigvals, eigvecs = np.linalg.eig(key)
    eigvalsprime, eigvecsprime = np.linalg.eig(keyprime)

    # to do later when the notation has been clarified... [i]x, [e]x

    #find largest eigenvalue
    largest_eigval = eigvals[0]
    for i in range(len(eigvals)):
        if abs(eigvals[i]) > largest_eigval:
            largest_eigval = eigvals[i]
            y = eigvec[i] #corresponding eigenvector to that largest eigenvalue

    largest_eigvalprime = eigvalsprime[0]
    for i in range(len(eigvalsprime)):
        if abs(eigvalsprime[i]) > largest_eigvalprime:
            largest_eigvalprime = eigvalsprime[i]
            y = eigvec[i] #corresponding eigenvector to that largest eigenvalue

    # at long last! z
    z = np.dot(Dinv, y)
    zprime = np.dot(Dinvprime, yprime)
    z = (z/np.linalg.norm(z) + zprime/np.linalg.norm(zprime))/2

    w = np.dot(e, z)
    wprime = np.dot(F, z)

    Hp = np.array([1, 0, 0,
                   0, 1, 0,
                   w[0], w[1], 1]).reshape(3,3)
    Hpprime = np.array([1, 0, 0,
                        0, 1, 0,
                        wprime[0], wprime[1], 1]).reshape(3,3)

    return w, wprime, Hp, Hpprime
