$(document).ready(function() {

    // 섹션 버튼 클릭
    $('#position_btn_container button').click(function() {
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
