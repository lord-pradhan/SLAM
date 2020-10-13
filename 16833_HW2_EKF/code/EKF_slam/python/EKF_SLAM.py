import numpy as np
import re
import matplotlib.pyplot as plt
np.set_printoptions(suppress=True, threshold=np.inf, linewidth=np.inf)


def drawCovEllipse(c, cov, setting):
    """Draw the Covariance ellipse given the mean and covariance

    :c: Ellipse center
    :cov: Covariance matrix for the state
    :returns: None

    """
    U, s, Vh = np.linalg.svd(cov)
    a, b = s[0], s[1]
    vx, vy = U[0, 0], U[0, 1]
    theta = np.arctan2(vy, vx)
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    phi = np.arange(0, 2*np.pi, np.pi/50)
    rot = []
    for i in range(100):
        rect = (np.array([3*np.sqrt(a)*np.cos(phi[i]), 3*np.sqrt(b)*np.sin(phi[i])]))[:, None]
        rot.append(R @ rect + c)

    rot = np.asarray(rot)
    plt.plot(rot[:, 0], rot[:, 1], c=setting, linewidth=0.75)


def drawTrajAndMap(X, last_X, P, t):
    """Draw Trajectory and map

    :X: Current state
    :last_X: Previous state
    :P: Covariance
    :t: timestep
    :returns: None

    """
    plt.ion()
    drawCovEllipse(X[0:2], P[0:2, 0:2], 'b')
    plt.plot([last_X[0], X[0]], [last_X[1], X[1]], c='b', linewidth=0.75)
    plt.plot(X[0], X[1], '*b')

    if t == 0:
        for k in range(6):
            drawCovEllipse(X[3 + k*2:3 + k*2+2], P[3 + k*2:3 + 2*k + 2, 3 + 2*k:3 + 2*k + 2], 'r')
    else:
        for k in range(6):
            drawCovEllipse(X[3 + k*2:3 + k*2+2], P[3 + 2*k:3 + 2*k + 2, 3 + 2*k:3 + 2*k + 2], 'g')

    plt.draw()
    plt.waitforbuttonpress(0)


def drawTrajPre(X, P):
    """ Draw trajectory for Predicted state and Covariance

    :X: Prediction vector
    :P: Prediction Covariance matrix
    :returns: None

    """
    drawCovEllipse(X[0:2], P[0:2, 0:2], 'm')
    plt.draw()
    plt.waitforbuttonpress(0)

def main():
    """Main function for EKF

    :arg1: TODO
    :returns: TODO

    """
    # TEST: Setup uncertainty parameters
    sig_x = 0.25
    sig_y = 0.1
    sig_alpha = 0.1
    sig_beta = 0.1
    sig_r = 0.16

    # Generate variance from standard deviation
    sig_x2 = sig_x**2
    sig_y2 = sig_y**2
    sig_alpha2 = sig_alpha**2
    sig_beta2 = sig_beta**2
    sig_r2 = sig_r**2

    # Open data file
    data_file = open("../../data/data.txt", 'r')

    # Read the first measurement data
    line = data_file.readline()
    fields = re.split('[\t ]', line)[:-1]
    arr = np.array([float(field) for field in fields])
    measure = arr[:, None]
    t = 1

    # Setup control and measurement covariance
    control_cov = np.diag([sig_x2, sig_y2, sig_alpha2])
    measure_cov = np.diag([sig_beta2, sig_r2])

    # Setup the initial pose vector and pose uncertainty
    pose = (np.array([0, 0, 0]))[:, None]
    pose_cov = np.diag([0.02**2, 0.02**2, 0.1**2])

    # TODO: Setup the initial landmark estimates landmark[] and covariance matrix landmark_cov[]
    # Hint: use initial pose with uncertainty and first measurement

    ##############################################################
    ################## Write your code here ######################
    ##############################################################

    # Setup state vector x with pose and landmark vector
    X = np.vstack((pose, landmark))

    # Setup covariance matrix P with pose and landmark covariance
    P = np.block([[pose_cov,           np.zeros((3, 2*k))],
                  [np.zeros((2*k, 3)),       landmark_cov]])

    # Plot initial state and covariance
    last_X = X
    drawTrajAndMap(X, last_X, P, 0)

    # Read file sequentially for controls
    # and measurements
    for line in data_file:
        fields = re.split('[\t ]', line)[:-1]
        arr = np.array([float(field) for field in fields])
        if arr.shape[0] == 2:
            d, alpha = arr[0], arr[1]
            control = (np.array([d, alpha]))[:, None]

            # TODO: Predict step
            # (Notice: predict state x_pre[] and covariance P_pre[] using input control data and control_cov[])

            ##############################################################
            ################## Write your code here ######################
            ##############################################################


            # Draw predicted state X_pre and Covariance P_pre
            drawTrajPre(X_pre, P_pre)

        # Read the measurement data
        else:
            measure = (arr)[:, None]

            # TODO: Correction step
            # (Notice: Update state X[] and covariance P[] using the input measurement data and measurement_cov[])

            ##############################################################
            ################## Write your code here ######################
            ##############################################################

            drawTrajAndMap(X, last_X, P, t)
            last_X = X
            t += 1

    # EVAL: Plot ground truth landmarks

    ##############################################################
    ################## Write your code here ######################
    ##############################################################



if __name__ == "__main__":
    main()
