# CCAT

CCAT is a system built for the Formula Student team BCN eMotorsport in order to **classify** and **keep track** of the cones detected from a global 3D map.
    
To fulfill this purpose, CCAT implements a **sensor fusion** model. It **registers** the bounding boxes obtained from the neural network that processes the camera images to the cones detected on the 3D map that LiDAR odometry provides.
    
CCAT must obtain a vector of classified cones. Each cone should have a type $t_{ccat}$ according to the class of cone they belong to. According to [FSG](https://www.formulastudent.de/fsg/) DE 6.2.4, possible types are _big orange_ ($BO$), _small orange_ ($SO$), _small yellow_ ($Y$) and _small blue_ ($B$). In order to also represent the case where cone type is unknown (due to camera range or uncertainty), type _unknown_ ($UNK$) will also be a possible type.

<p align="center">
  <img src="./documentation/assets/cones_unk_background.png" alt="CCAT cone types" width="500" /><br />
  CCAT cone types
</p>

In addition, CCAT will make sure (as far as possible) that the cones it outputs agree with the ground truth. To do so, it will track every cone over time to robustly assign them a unique $id_{ccat}$.
    
We will set/change the decisions on the set of tracked cones only when a certain confidence is reached since, later, other algorithms will take these cones (and identify them through the $id_{ccat}$) and that will decide which path will the car take.

## Input
- An **observation vector** from the global 3D map, this includes the cone position (centroid) $p_{obs}\, (\mathbb{R}^3)$, a detection confidence $\alpha_{obs}$ and the cone's point cloud $pcl_{obs}$.\
  This is: {$(p_{obs},\alpha_{obs},pcl_{obs})$}.
- Car \textbf{location} and \textbf{heading}, a pose 6D vector.\
  This is: $(x, y, z, \phi, \theta, \psi)$.
- Left and right camera detected \textbf{bounding boxes} of cones, including the type of cone and confidence.\
  This is: {$(x_{bb_0}, y_{bb_0}, x_{bb_1}, y_{bb_1}, t_{bb}, \alpha_{bb})$}.

Note: In order to synchronize the data afterwards, all input messages are timestamped.

## Output
- **Classified** cone vector with class confidence and each cone will have a unique id (between iterations).\
This is: {$(p_{ccat},t_{ccat},id_{ccat},\alpha_{ccat})\mid p_{ccat} \in \mathbb{R}^3, t_{ccat} \in (Y,B,SO,BO,UNK)$}

## Result
<p align="center">
  <img src="./documentation/assets/execution.gif" alt="CCAT projections and markers" /><br />
  CCAT left projections, markers and right projections
</p>
