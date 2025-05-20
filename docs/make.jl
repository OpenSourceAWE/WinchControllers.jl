using WinchControllers
using Documenter

# DocMeta.setdocmeta!(WinchControllers, :DocTestSetup; recursive=true)

makedocs(;
    modules=[WinchControllers],
    authors="Uwe Fechner <fechner@aenarete.eu>",
    repo="https://github.com/OpenSourceAWE/WinchControllers.jl/blob/{commit}{path}#{line}",
    sitename="WinchControllers.jl",
    format=Documenter.HTML(;
        repolink = "https://github.com/OpenSourceAWE/WinchControllers.jl",
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://OpenSourceAWE.github.io/WinchControllers.jl",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
        "Generic Components" => "components.md",
        "Winchcontroller" => "winchcontroller.md",
        "Functions and Macros" => "functions.md",
        "Tests" => "tests.md",
    ],
)

deploydocs(;
    repo="github.com/OpenSourceAWE/WinchControllers.jl",
    devbranch="main",
)
