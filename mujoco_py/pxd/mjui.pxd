cdef extern from "mjui.h" nogil:
    # Global constants
    enum: mjMAXUISECT
    enum: mjMAXUIITEM
    enum: mjMAXUITEXT
    enum: mjMAXUINAME
    enum: mjMAXUIMULTI
    enum: mjMAXUIEDIT
    enum: mjMAXUIRECT
    enum: mjSEPCLOSED


    # predicate function: set enable/disable based on item category
    ctypedef int (*mjfItemEnable)(int category, void* data);

    ctypedef struct mjuiState:               # mouse and keyboard state
        # constants set by user
        int nrect                   # number of rectangles used
        mjrRect rect[mjMAXUIRECT]   # rectangles (index 0: entire window)
        void* userdata              # pointer to user data (for callbacks)

        # event type
        int type                    # (type mjtEvent)

                                    # mouse buttons
        int left                    # is left button down
        int right                   # is right button down
        int middle                  # is middle button down
        int doubleclick             # is last press a double click
        int button                  # which button was pressed (mjtButton)
        double buttontime           # time of last button press

                                    # mouse position
        double x                    # x position
        double y                    # y position
        double dx                   # x displacement
        double dy                   # y displacement
        double sx                   # x scroll
        double sy                   # y scroll

                                    # keyboard
        int control                 # is control down
        int shift                   # is shift down
        int alt                     # is alt down
        int key                     # which key was pressed
        double keytime              # time of last key press

                                    # rectangle ownership and dragging
        int mouserect               # which rectangle contains mouse
        int dragrect                # which rectangle is dragged with mouse
        int dragbutton              # which button started drag (mjtButton)

    ctypedef struct mjuiThemeSpacing:        # UI visualization theme spacing
        int total              # total width
        int scroll             # scrollbar width
        int label              # label width
        int section            # section gap
        int itemside           # item side gap
        int itemmid            # item middle gap
        int itemver            # item vertical gap
        int texthor            # text horizontal gap
        int textver            # text vertical gap
        int linescroll         # number of pixels to scroll
        int samples            # number of multisamples


    ctypedef struct mjuiThemeColor:          # UI visualization theme color
        float master[3]            # master background
        float thumb[3]             # scrollbar thumb
        float secttitle[3]         # section title
        float sectfont[3]          # section font
        float sectsymbol[3]        # section symbol
        float sectpane[3]          # section pane
        float shortcut[3]          # shortcut background
        float fontactive[3]        # font active
        float fontinactive[3]      # font inactive
        float decorinactive[3]     # decor inactive
        float decorinactive2[3]    # inactive slider color 2
        float button[3]            # button
        float check[3]             # check
        float radio[3]             # radio
        float select[3]            # select
        float select2[3]           # select pane
        float slider[3]            # slider
        float slider2[3]           # slider color 2
        float edit[3]              # edit
        float edit2[3]             # edit invalid
        float cursor[3]            # edit cursor


    ctypedef struct mjuiItem:                # UI item
        # common properties
        int type                   # type (mjtItem)
        char name[mjMAXUINAME]     # name
        int state                  # 0: disable, 1: enable, 2+: use predicate
        void *pdata                # data pointer (type-specific)
        int sectionid              # id of section containing item
        int itemid                 # id of item within section

        # internal
        mjrRect rect               # rectangle occupied by item


    ctypedef struct mjuiSection:             # UI section
        # properties
        char name[mjMAXUINAME]      # name
        int state                   # 0: closed, 1: open
        int modifier                # 0: none, 1: control, 2: shift; 4: alt
        int shortcut                # shortcut key; 0: undefined
        int nitem                   # number of items in use
        mjuiItem item[mjMAXUIITEM]  # preallocated array of items
                                    # internal
        mjrRect rtitle              # rectangle occupied by title
        mjrRect rcontent            # rectangle occupied by content


    ctypedef struct mjUI:                    # entire UI
        # constants set by user
        mjuiThemeSpacing spacing    # UI theme spacing
        mjuiThemeColor color        # UI theme color
        mjfItemEnable predicate     # callback to set item state programmatically
        void* userdata              # pointer to user data (passed to predicate)
        int rectid                  # index of this ui rectangle in mjuiState
        int auxid                   # aux buffer index of this ui
        int radiocol                # number of radio columns (0 defaults to 2)

        # UI sizes (framebuffer units)
        int width                   # width
        int height                  # current heigth
        int maxheight               # height when all sections open
        int scroll                  # scroll from top of UI

                                    # mouse focus
        int mousesect               # 0: none, -1: scroll, otherwise 1+section
        int mouseitem               # item within section
        int mousehelp               # help button down: print shortcuts

                                    # keyboard focus and edit
        int editsect                # 0: none, otherwise 1+section
        int edititem                # item within section
        int editcursor              # cursor position
        int editscroll              # horizontal scroll
        char edittext[mjMAXUITEXT]  # current text
        mjuiItem* editchanged       # pointer to changed edit in last mjui_event

                                        # sections
        int nsect                       # number of sections in use
        mjuiSection sect[mjMAXUISECT]   # preallocated array of sections


    ctypedef struct mjuiDef:
        int type                   # type (mjtItem); -1: section
        char name[mjMAXUINAME]     # name
        int state                  # state
        void* pdata                # pointer to data
        char other[mjMAXUITEXT]    # string with type-specific properties
