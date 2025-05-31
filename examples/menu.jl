using REPL.TerminalMenus

options = ["test_mixer2 = include(\"../test/test_mixer2.jl\")",
           "test_mixer3 = include(\"../test/test_mixer3.jl\")",
           "test_solver = include(\"../test/test_solver.jl\")",
           "test_speedcontroller1 = include(\"../test/test_speedcontroller1.jl\")",
           "test_speedcontroller2 = include(\"../test/test_speedcontroller2.jl\")",
           "test_forcespeedcontroller1 = include(\"../test/test_forcespeedcontroller1.jl\")",
           "test_forcespeedcontroller2 = include(\"../test/test_forcespeedcontroller2.jl\")",
           "test_winchcontroller = include(\"test_winchcontroller.jl\")",
           "autotune_ = include(\"../scripts/autotune.jl\")",
           "test_tuned_winchcontroller = include(\"test_tuned_winchcontroller.jl\")",
           "plot_power = include(\"plot_power.jl\")",
           "quit"]

function example_menu()
    active = true
    while active
        menu = RadioMenu(options, pagesize=8)
        choice = request("\nChoose function to execute or `q` to quit: ", menu)

        if choice != -1 && choice != length(options)
            eval(Meta.parse(options[choice]))
        else
            println("Left menu. Press <ctrl><d> to quit Julia!")
            active = false
        end
    end
end

example_menu()