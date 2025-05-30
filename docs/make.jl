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
        mathengine = Documenter.MathJax(),
    ),
    pages=[
        "Home" => "index.md",
        "Generic Components" => "components.md",
        "Functions and Macros" => "functions.md",
        "Winchcontroller" => "winchcontroller.md",
        "Winchcontroller Settings" => "settings.md",
        "Performance Indicators" => "performance_indicators.md",
        "Autotuning" => "autotuning.md",
        "Tests" => "tests.md",
    ],
)

deploydocs(;
    repo="github.com/OpenSourceAWE/WinchControllers.jl",
    devbranch="main",
)
