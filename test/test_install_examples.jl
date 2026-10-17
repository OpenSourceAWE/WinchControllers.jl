# Tests of copy_examples() and install_examples(). They run in a temporary folder and, for
# install_examples(), in a temporary environment; the working directory, the active project
# and the data path of KiteUtils are restored afterwards.
using Pkg, Test, WinchControllers, KiteUtils

@testset "copy_examples" begin
    old_dir = pwd()
    mktempdir() do dir
        try
            cd(dir)
            files = WinchControllers.copy_examples()
            examples = readdir(joinpath(dir, "examples"))
            @test "menu.jl" in files
            @test "test_winchcontroller.jl" in examples
            # the Project.toml of the examples loads WinchControllers from the parent folder
            @test ! ("Project.toml" in examples)
            @test ! any(startswith("Manifest"), examples)
        finally
            cd(old_dir)
        end
    end
end

@testset "install_examples" begin
    old_dir, old_data, old_project = pwd(), get_data_path(), dirname(Base.active_project())
    mktempdir() do dir
        try
            cd(dir)
            Pkg.activate(dir; io=devnull)
            # only add the packages; precompiling them is not needed for this test
            withenv("JULIA_PKG_PRECOMPILE_AUTO" => "0") do
                WinchControllers.install_examples()
            end
            @test isfile(joinpath(dir, "examples", "menu.jl"))
            @test isfile(joinpath(dir, "data", "wc_settings.yaml"))
            @test get_data_path() == joinpath(dir, "data")
            # all packages that the examples need are added
            deps = keys(Pkg.project().dependencies)
            examples_project = Pkg.TOML.parsefile(joinpath(pkgdir(WinchControllers), "examples", "Project.toml"))
            for name in keys(examples_project["deps"])
                name == "WinchControllers" && continue
                @test name in deps
            end
        finally
            Pkg.activate(old_project; io=devnull)
            cd(old_dir)
            set_data_path(old_data)
        end
    end
end
