function error_if_out_of_time(start_tic, timeout)
    if toc(start_tic) > timeout
        error('Out of time!')
    end
end