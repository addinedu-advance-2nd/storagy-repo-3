$(document).ready(function() {

    // Search Group Button Click
    $('.section-group__button').click(function() {
        var value = $(this).val();
        
        $.ajax({
            url: `/goal_position/${value}`,
            type: "GET",
            success: function(response) {
                console.log("Response from server:", response);
            },
            error: function(error) {
                console.error("Error:", error);
            }
        });
    });

});
