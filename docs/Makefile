SPHINXOPTS    =
SPHINXBUILD   = python -msphinx
SPHINXPROJ    = mujoco-py
SOURCEDIR     = .
BUILDDIR      = build

.PHONY: all clean

all:
	@$(SPHINXBUILD) -M html "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

clean:
	@$(SPHINXBUILD) -M clean "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
