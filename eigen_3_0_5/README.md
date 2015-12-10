#Eigen 3.0.5

This repository has been created to keep a backup copy of the Eigen 3.0.5 linear algebra template library to maintain support for [`ocra-core`](https://github.com/ocra-recipes/ocra-core), namely the unsupported LGSM (Lie Group Solid Mechanics) library. The Eigen project can be found at http://eigen.tuxfamily.org/index.php?title=Main_Page. 

##Install

Because Eigen consists entirely of header files there is no compilation to be done, you simply have to put everything in the "right" place.

So we start by deciding where we want to put our version of Eigen. If you already have other versions of Eigen installed then I would recommend putting Eigen somewhere local like: `/home/user/...` but this is up to you. For demonstration purposes let's say our user *bob* decides to put Eigen in `/home/bob/code`

First he clones the repo:

```bash
cd /home/bob/code
git clone https://github.com/ocra-recipes/eigen_3_0_5
```
Let's now call `/home/bob/code/eigen_3_0_5/`, `$EIGEN_ROOT`.

Now he just needs to make sure package config can find these headers. First he opens `eigen3.pc` and changes the `CFlags` to point to `$EIGEN_ROOT`:

```pc
Cflags: -I$EIGEN_ROOT -I$EIGEN_ROOT/unsupported
```
becomes...
```pc
Cflags: -I/home/bob/code/eigen_3_0_5 -I/home/bob/code/eigen_3_0_5/unsupported
```

Great! Now he just has to tell pkgconfig about this new file. To do so there he has two options:

1. Copy the eigen3.pc file to the pkgconfig folder.

```bash
sudo cp $EIGEN_ROOT/eigen3.pc /usr/share/pkgconfig
```

2. Add `$EIGEN_ROOT` to the pkgconfig path in the `.bashrc`.

```bash
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/home/bob/code/eigen_3_0_5
```

And that is it.

##Warning
If you already have Eigen 3.XX then this will cause a direct conflict. You have been warned!

