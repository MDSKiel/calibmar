# COLMAP underwater fork
The underwater colmap fork at https://cau-git.rz.uni-kiel.de/inf-ag-koeser/colmap_underwater.git is used as a library, integrated via git subtree.
## Updating
Use squashing pull from git subtree, such that local changes can be preserved and upstream changes merged.
`git subtree pull --prefix lib/colmap  https://cau-git.rz.uni-kiel.de/inf-ag-koeser/colmap_underwater.git underwater --squash'