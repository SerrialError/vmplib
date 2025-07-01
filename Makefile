# Compiler and flags
CXX      := g++
CXXFLAGS := -g -std=c++17 -Wall -Wextra -Iinclude

# Directories
SRCDIR := src
INCDIR := include
OBJDIR := obj
BINDIR := bin

# Grab all .cpp files under src/
SRCS := $(wildcard $(SRCDIR)/*.cpp)

# Turn “src/foo.cpp” into “obj/foo.o”
OBJS := $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR)/%.o,$(SRCS))

# Final executable
EXE := $(BINDIR)/main

# Default target: make sure bins + objs exist, then build the exe
all: $(BINDIR) $(OBJDIR) $(EXE)

# Link step: link all .o files into bin/main
$(EXE): $(OBJS)
	$(CXX) $^ -o $@

# Compile step: for each src/%.cpp → obj/%.o
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Create bin/ and obj/ if they don’t exist
$(OBJDIR) $(BINDIR):
	mkdir -p $@

# “make clean”
.PHONY: clean
clean:
	rm -rf $(OBJDIR) $(BINDIR)

# Mark “all” as phony so it always runs
.PHONY: all
