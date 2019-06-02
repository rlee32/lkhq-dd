CXX = g++-8
CXX_FLAGS = -std=c++17
CXX_FLAGS += -Wuninitialized -Wall -Wextra -Werror -pedantic -Wfatal-errors # source code quality.
CXX_FLAGS += -O3 -ffast-math # "production" version.
#CXX_FLAGS += -O0 -g # debug version.
CXX_FLAGS += -I./ # include paths.

LINK_FLAGS = -lstdc++fs # filesystem

SRCS = k-opt.cpp Tour.cpp \
	RandomFinder.cpp \
	OptimalFinder.cpp \
    point_quadtree/Node.cpp \
    point_quadtree/point_quadtree.cpp \
    point_quadtree/PointInserter.cpp \
    cycle_check.cpp

%.o: %.cpp; $(CXX) $(CXX_FLAGS) -o $@ -c $<

OBJS = $(SRCS:.cpp=.o)

all: $(OBJS); $(CXX) $^ $(LINK_FLAGS) -o k-opt.out

clean: ; rm -rf k-opt.out $(OBJS) *.dSYM
