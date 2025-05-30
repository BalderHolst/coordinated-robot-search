#!/usr/bin/env bash

function convert() {
    # Create a temporary LaTeX header file
    header_file=$(mktemp)

    cat > "$header_file" << EOF
\\usepackage{titling}
\\pretitle{\\begin{center}\\Huge}
\\posttitle{\\end{center}}
EOF

    pandoc \
        --pdf-engine=xelatex \
        --variable mainfont="DejaVu Serif" \
        --variable fontsize=11pt \
        --variable geometry="margin=1.5cm" \
        --variable linkcolor=blue \
        --highlight-style=tango \
        --metadata title="$3" \
        --include-in-header="$header_file" \
        -f markdown -t pdf $1 -o $2

    echo "Converted $1 to $2"
}

cd ..
convert README.md report/appendix/README.pdf "Repository README"
