## SOLID - Software Library for Interference Detection

Копия библиотеки [solid3](https://github.com/dtecta/solid3) для 
проверки коллизий двух полигональных моделей

SOLID is a software library containing functions for performing
intersection tests and proximity queries that are useful in the context
of collision detection. Collision detection is the process of detecting
pairs of geometric objects that are intersecting or are within a given
proximity of each other. In particular, SOLID is useful for detecting
collisions between objects that are moving relatively of each other over
time. The motions of objects are controlled by the client application,
and are not determined or affected by SOLID. 

This open-source edition of SOLID version 3 is released under the terms of
either the GNU Public License (GPL) or the Q Public License (QPL). This means
that for software created with SOLID version 3 you must comply with the terms
of one of these licenses. You may choose which of these licenses best suits
your purpose. See the following files contained in this distribution for a
complete list of terms and conditions of these licenses:  

		 LICENSE_QPL.txt	 The Q Public License 
		 LICENSE_GPL.txt	 The GNU General Public License

These licenses do not permit the use of SOLID 3 in closed-source software
products. For enquiries about commercial use of SOLID, please contact
info@dtecta.com.    

SOLID 3 uses Qhull from The Geometry Center of the University of Minnesota.
Qhull is copyrighted as noted below.  Qhull is free software and may be
obtained via anonymous ftp from geom.umn.edu.   
        
                    Qhull, Copyright (c) 1993-2002

       The National Science and Technology Research Center for
        Computation and Visualization of Geometric Structures
                        (The Geometry Center)
                       University of Minnesota
                            400 Lind Hall
                        207 Church Street S.E.
                      Minneapolis, MN 55455  USA

                       email: qhull@geom.umn.edu

Installation
------------

SOLID builds can be configured using CMake for all platforms. For details on 
how to install SOLID using autoconf tools see the documentation in the 'doc'
directory. 

Platforms
---------

SOLID 3 has been tested on the following platforms:

    Linux x86, x86_64	gcc 2.95, gcc 3.3, gcc 3.4, gcc 4.8
	Win32		        MSVC++ 6.0 up to MSVC++ 14.0 


Some of the example applications use GLUT. GLUT is a utility toolkit for
creating OpenGL applications. The original source code for GLUT is Copyright
1997 by Mark J. Kilgard. FreeGLUT, a compatible GLUT implementation can be downloaded from http://freeglut.sourceforge.net/