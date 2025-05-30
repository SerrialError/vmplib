# Compiler and flags
CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Iinclude

# Directories
SRCDIR := src
INCDIR := include
OBJDIR := obj
BINDIR := bin

# Sources
SRCS := $(SRCDIR)/main.cpp \
        $(SRCDIR)/2dmpsimulate.cpp

# Objects (in obj/)
OBJS := $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR)/%.o,$(SRCS))

# Single executable
EXE := $(BINDIR)/main

# Default target
all: $(BINDIR) $(OBJDIR) $(EXE)

# Link the final binary from both .o files
$(EXE): $(OBJS)
	$(CXX) $^ -o $@

# Compile each .cpp → .o
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Create dirs if needed
$(OBJDIR) $(BINDIR):
	mkdir -p $@

# Clean up
.PHONY: clean
clean:
	rm -rf $(OBJDIR) $(BINDIR)

# Phony for default
.PHONY: all
