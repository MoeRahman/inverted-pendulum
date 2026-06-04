# ====================================================================
# STYLES.GP - Reusable Dark Theme Profile
# ====================================================================

# 1. Background (Uses the screen object trick to work on ALL terminals)
set object 999 rectangle from screen 0,0 to screen 1,1 fillstyle solid 1.0 fillcolor rgb '#0A0A0A' behind

# 2. Borders and Grid Lines
set border 31 lw 1.5 lc rgb '#33FF33'     # White outer border frame
set grid xtics ytics lc rgb '#33FF33' lw 2 # Subtle grey grid lines

# 3. Typography and Labels
set title font "Sans, 24" textcolor rgb '#33FF33'
set ylabel font "Sans, 18" textcolor rgb '#33FF33'
set xlabel font "Sans, 18" textcolor rgb '#33FF33'
set key font "Sans, 24" textcolor rgb '#33FF33'
set tics textcolor rgb '#33FF33'

# 4. Vibrant Line Styles (High contrast for dark backgrounds)
set style line 1 lc rgb '#33FF33' lw 5 pt 7 ps 1.5

# Line 2: Soft White, Dashed Style
set style line 2 lc rgb '#ff00ff' lw 5 dt 2

# Line 3: Retro Orange Points (Line type 0 hides the connecting line)
set style line 3 lc rgb '#33FF33' pt 7 ps 0.1 dt 1

# Line 4: Neon Red, Solid Line
set style line 4 lc rgb '#E63946' lw 5 dt 2

# Line 2: Soft White, Dashed Style
set style line 5 lc rgb '#00ffbb' lw 5