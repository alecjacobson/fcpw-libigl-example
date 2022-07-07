# Example project mixing libigl and fcpw


    git clone --recursive https://github.com/alecjacobson/fcpw-libigl-example/

## Run

    ./fcpw-test [path-to-mesh] 

    ./fcpw-test [path-to-mesh] [path-to-queries].dmat

Will output 

    - `Q.dmat`  #Q by 3 list of query points
    - `I.dmat`  #I list of indices into triangles
    - `UV.dmat`  #Q by 2 list of barycentric coordinates

