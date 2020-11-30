### 2D Clustering:
* [Clustering](https://en.wikipedia.org/wiki/Cluster_analysis) in general is the task of grouping a set of objects in such a way that objects in the same group are similar to each other than those in other groups.
* The benefit of using a 2D-Tree in clustering is to search and identify clusters efficiently.

#### [KD-Tree](https://en.wikipedia.org/wiki/K-d_tree):
* It is a space-partitioning data structure for organizing points in a k-dimensional space.
###### [Construction](https://youtu.be/LdaL-l2S76c):
  - Let's consider a 2D Tree.
  * The tree has a root and it is the first point in the tree and located in level 0.
  * Each point will be called a node, and have a left and right nodes.
  * Choosing points to insert is done randomly or by picking the median w.r.t the values in the i<sup>th</sup> axis, where i is the depth % no. of axes, or any other axis than the previously chosen.
  * Traversing the tree by either going left or right is done by checking if value of xp >= xr then you go right, otherwise go left. Where p is the inserted or the considered point and r is the root which splits the space in the x axis h at this level.
  * It can be summarized to three steps:
  pick, split, repeat.

  ###### Searching:
    * Searching for nearby points using a point (p) is done by traversing the tree with the same procedure as explained above, narrowing down the possible candidates with the cluster threshold.    

#### Euclidean Clustering:
* Uses the euclidean distance between the target point and the neighboring candidates as the check condition.

#### Result:
 <img src="images/clustering2D.png" alt="drawing" width="600" height="400"/>
