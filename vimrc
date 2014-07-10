"----- pridkett's .vimrc file
"----- designed for vim 5.3
"----- october 10th, 1998

"----- set up the stuff for color highlighing in an xterm
if &term =~ "xterm"
 if has("terminfo")
  set t_Co=16
  set t_Sf=[3%p1%dm
  set t_Sb=[4%p1%dm
  set t_vb=
  set t_kh=[7%p1%dm
  set t_@7=[4%p1%dm
 else
  set t_Co=16
  set t_Sf=[3%dm
  set t_Sb=[4%dm
  set t_vb=
  set t_kh=[7%dm
  set t_@7=[4%dm
 endif
endif

  "set t_kP=pageUp
  "set t_kN=pageDown
  "set t_kh=home
  "set t_@7=end
"----- only turn on syntax highlighting of there are more than one color
syntax on
"----- autoindenting is good
set autoindent
set smartindent
"----- don't don't out in insert mode
set noinsertmode
"----- allow us to backspace before an insert
set backspace=2
"----- tabs are to be set at 4 spaces
set cinoptions=>4
set tabstop=4
set shiftwidth=4
set softtabstop=4
"----- I guess I like real tab characters!
set noexpandtab
"----- show the ruler for editing
set ruler
"----- turn off the mouse in the xterm
set mouse=
"----- show the command in the status line
set showcmd
"---- STOP BEEPING!
set noerrorbells
"---- show matching brackets
set showmatch
"---- smoother output
set ttyfast

"----- always have a status line
set laststatus=2

"----- Let's try the following settings for C/C++
autocmd FileType c,cpp
\	set formatoptions=croql
\	cindent
\  comments=sr:/*,mb:*,ex:*/,://

"----- We need real tabs for Makefiles.
autocmd FileType make set noexpandtab
autocmd FileType make set nosmarttab

"----- have java highlight our functions
"let java_highlight_functions=1

"----- have php3 highlight SQL, and keep in better sync.
let php3_sql_query = 1
let php3_minlines = 3000
let php3_baselib = 1

highlight Comment     NONE
highlight Constant    NONE
highlight Delimiter   NONE
highlight Directory   NONE
highlight Error       NONE
highlight ErrorMsg    NONE
highlight Identifier  NONE
highlight LineNr      NONE
highlight ModeMsg     NONE
highlight MoreMsg     NONE
highlight Normal      NONE
highlight NonText     NONE
highlight PreProc     NONE
highlight Question    NONE
highlight Search      NONE
highlight Special     NONE
highlight SpecialKey  NONE
highlight Statement   NONE
highlight StatusLine  NONE
highlight Title       NONE
highlight Todo        NONE
highlight Type        NONE
highlight Visual      NONE
highlight WarningMsg  NONE
highlight Folded	    NONE
highlight FoldColumn  NONE
highlight SignColumn  NONE
"----- these are the new superior colors
set background=dark
highlight Comment      ctermfg=Red
highlight Constant     ctermfg=Gray
highlight Delimiter    ctermfg=DarkGray
highlight Directory    ctermfg=Yellow
highlight Error        ctermfg=White ctermbg=Red
highlight ErrorMsg     ctermfg=Yellow
highlight Identifier   ctermfg=DarkBlue
highlight IncSearch    term=reverse cterm=reverse gui=reverse
highlight LineNr       ctermfg=Red
highlight ModeMsg      ctermfg=White ctermbg=DarkRed
highlight MoreMsg      ctermfg=White ctermbg=DarkRed
highlight NonText      ctermfg=LightGreen
highlight Normal       ctermfg=White
highlight PreProc      ctermfg=Yellow
highlight Question     ctermfg=Red
highlight Search       ctermfg=White ctermbg=DarkBlue
highlight Special      ctermfg=Yellow
highlight SpecialKey   cterm=bold 
highlight Statement    ctermfg=Green
highlight StatusLine   ctermfg=Yellow ctermbg=Blue
highlight StatusLineNC ctermfg=Yellow ctermbg=Black
highlight Title        ctermfg=Red
highlight Todo         ctermfg=Yellow ctermbg=Black
highlight Type         ctermfg=Magenta
highlight VertSplit    term=reverse cterm=reverse gui=reverse
highlight Visual       cterm=reverse
highlight VisualNOS    term=bold,underline cterm=bold,underline gui=bold,underline
highlight WarningMsg   ctermfg=Red
highlight WildMenu     term=standout ctermfg=0 ctermbg=11 guifg=Black guibg=Yellow
highlight Folded       term=standout ctermfg=14 ctermbg=8 guifg=Cyan guibg=DarkGrey
highlight FoldColumn   term=standout ctermfg=14 ctermbg=8 guifg=Cyan guibg=Grey
highlight DiffAdd      term=bold ctermbg=4 guibg=DarkBlue
highlight DiffChange   term=bold ctermbg=5 guibg=DarkMagenta
highlight DiffDelete   term=bold ctermfg=12 ctermbg=6 gui=bold guifg=Blue guibg=DarkCyan
highlight DiffText     term=reverse cterm=bold ctermbg=12 gui=bold guibg=Red
highlight SignColumn   term=standout ctermfg=14 ctermbg=8 guifg=Cyan guibg=Grey
highlight Cursor       guifg=bg guibg=fg
highlight lCursor      guifg=bg guibg=fg
highlight Underlined   term=underline cterm=underline ctermfg=12 gui=underline guifg=#80a0ff
highlight Ignore       ctermfg=Gray guifg=bg

"
"edit my .vimrc file
map ,v :e ~/.vimrc
"update the system settings from my vimrc file
map ,u :source ~/.vimrc

"handy pasting functions.
map ,i :set nosmartindent<CR>:set noautoindent<CR>
map ,I :set smartindent<CR>:set autoindent<CR>

"set tw and unset it
map ,w :set tw=76<CR>
map ,W :set tw=0<CR>

"handy tscope binding
map tt <SPACE>byw:!tscope -i ' f <C-R>"'<CR>

set tags=./tags

"set tags=~/.tags_fabos,~/.tags_fabos_bccb,~/.tags_fabos_bfos,~/.tags_fabos_cntcp,~/.tags_fabos_dfos,~/.tags_fabos_linux,~/.tags_fabos_src,~/.tags_fabos_tps,~/.tags_nist_ctr_drbg_with_df

set viminfo='50,\"1000,:100,n~/.viminfo
map ,t :'a,'b!tf<CR>
set shortmess=aIt

"----- instructions for VIM on processing this file
" vim:ts=3
